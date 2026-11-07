# Voorstel: MQTT vervangen door HTTP POST voor analytics

Status: voorstel · Datum: 2026-11-08

> **Update:** op branch `feature/tiny-mqtt` is **optie B** uitgewerkt: de external component
> `src/openamber/tiny_mqtt`. Die publiceert QoS 0 naar dezelfde broker en topics, maar zet alleen
> een verbinding op als er iets te versturen is (DNS → CONNECT/CONNACK → PUBLISH… → 2 s linger →
> DISCONNECT). Er draait geen extra taak, er wordt geen MQTT-object per entity aangemaakt, en
> buffers en socket worden na elke sessie vrijgegeven. De server hoeft niet te veranderen.

## Samenvatting

De `mqtt:` component wordt in OpenAmber alleen gebruikt voor de opt-in analytics: elke 5 minuten
13 JSON-berichten naar `mqtt.openamber.nl`, realtime foutcodes, en de EEPROM-dump. Voor dat beperkte
gebruik kost MQTT veel geheugen. **Ongeveer 22 KB statisch DRAM gaat altijd op, ook als analytics uit
staat**, en zodra analytics aan staat komt daar nog ongeveer 10 KB permanente heap bij.

Voorstel: haal `mqtt:` weg en verstuur de analytics als **één HTTP POST per interval** via de
`http_request` component. Die zit al in de firmware (OTA, firmware-update, ophalen van de device-id).
De heap wordt dan alleen tijdens een request gebruikt en daarna direct weer vrijgegeven.

## Huidige situatie

| Gebruik | Waar | Hoe |
|---|---|---|
| Periodieke analytics (13 berichten, retained, QoS 0) | `analytics_scripts.yaml` → `send_analytics` | `mqtt.publish_json` × 13, 150 ms tussenpozen |
| Realtime foutcodes | `send_errors_realtime` | `mqtt.publish_json` |
| EEPROM-dump (5 chunks) | `dump_eeprom_parameters` | `id(mqtt_client).publish(...)` + `is_connected()` |
| Verbinden / device-id ophalen | `openamber.yaml` `on_boot` / `on_connect` | `mqtt.enable`, `on_connect` → `fetch_analytics_device_id` (dat is al HTTP) |

Er is geen `on_message`/subscribe, er wordt niets binnen ontvangen, en discovery, log-topic en
`topic_prefix` staan uit. MQTT wordt hier dus puur als transport naar buiten gebruikt.

## Waar het geheugen naartoe gaat

Gemeten op de huidige ESP32-S3 build (`src/.esphome/build/openamber/build/openamber.elf`, ESPHome
2026.10.2) met `xtensa-esp32s3-elf-nm -S`:

### 1. Statisch (.bss in DRAM), altijd, ook als analytics uit staat

ESPHome maakt **voor elke entity automatisch een `MQTT*Component`-object aan** zodra er een `mqtt:`
blok in de config staat (`cv.OnlyWith(CONF_MQTT_ID, "mqtt")`). Door `topic_prefix: null` worden ze
tijdens setup als "internal" gemarkeerd en doen ze verder niets. Het geheugen ervoor wordt wel
gereserveerd.

| Object | Aantal | Grootte |
|---|---:|---:|
| `MQTTNumberComponent` | 131 | 36 B |
| `MQTTBinarySensorComponent` | 93 | 36 B |
| `MQTTSensorComponent` | 90 | 44 B |
| `MQTTSwitchComponent` | 51 | 36 B |
| `MQTTTextSensor` | 23 | 36 B |
| `MQTTSelectComponent` | 18 | 36 B |
| `MQTTPublishJsonAction` | 13 | 48 B |
| `MQTTClimateComponent` | 6 | 540 B |
| overige (button, time, text, datetime, update, trigger) | 13 | ~36 B |
| **Subtotaal entity-objecten** | **438** | **20 960 B** |
| `MQTTClientComponent` | 1 | 1 360 B |
| **Totaal statisch** | | **≈ 22,3 KB** |

Daar komen nog de registratieslots van deze ~438 extra componenten in de component-lijst van
`App` bij (ongeveer 4 B per stuk, ~1,7 KB, geschat). Bij elke `loop()` worden ze ook allemaal
doorlopen.

Op de **ESP32 zonder PSRAM** (`openamber-leejow.yaml`, `esp32dev`) gaat dit rechtstreeks van de
beschikbare heap af. Daar is de winst dus het grootst.

### 2. Runtime, alleen als analytics aan staat (permanent zolang verbonden)

Standaardwaarden van esp-mqtt (ESPHome stelt ze niet bij):

| Onderdeel | Geschat |
|---|---:|
| esp-mqtt taak-stack (`MQTT_TASK_STACK_SIZE` default) | ~6 KB |
| In- en uitgaande buffer (`buffer_size` / `out_buffer_size` default 1024) | ~2 KB |
| Client-struct, transport, outbox, ESPHome event-pool | ~1 KB |
| Persistente TCP-socket (lwIP PCB + data in de send-buffer) | 0,3–1 KB |
| **Totaal** | **≈ 9–10 KB** |

Deze heap blijft continu bezet en de keep-alive houdt de verbinding de hele tijd open. Doordat dit
geheugen naast de LVGL-, web_server- en EEBUS-allocaties staat, wordt de heap ook sneller
gefragmenteerd.

### 3. Flash

Ongeveer 35 KB code in ESPHome's MQTT-component, plus de esp-mqtt library met de SSL- en
WebSocket-transports (`CONFIG_MQTT_TRANSPORT_*`). Dat telt niet mee voor de heap, maar het scheelt
wel ruimte in de OTA-partitie.

## Voorgestelde oplossing: HTTP POST via `http_request`

### Werking

1. `mqtt:` en `mqtt.enable` verdwijnen helemaal uit `openamber.yaml`.
2. `send_analytics` bouwt **één** JSON-body met dezelfde korte keys als nu (`ts`, `fw`, `a1`, `h1`, `ua`, …)
   en verstuurt die als `POST http://openamber.nl/api/analytics/<mac>`. De server kan de keys dan 1-op-1
   overnemen.
3. `send_errors_realtime` → dezelfde endpoint, met alleen `ts` en de `e21xx`-velden. De bestaande
   debounce van 200 ms blijft.
4. EEPROM-dump → `POST .../eeprom_dump/<chunk>` per chunk, of alle 5 chunks samen in één body.
   In plaats van `is_connected()` kijkt de code naar `network::is_connected()` en naar het
   HTTP-statuscodeveld.
5. `fetch_analytics_device_id` blijft bestaan. In plaats van `on_connect` draait die dan na de
   eerste geslaagde POST (of zoals nu aan het eind van `send_analytics`).

Schets (`analytics_scripts.yaml`):

```yaml
- id: send_analytics
  then:
    - lambda: id(analytics_batch_timestamp) = id(my_time).timestamp_now();
    - http_request.post:
        url: !lambda 'return "http://openamber.nl/api/analytics/" + id(mac_address).state;'
        request_headers:
          Authorization: "Bearer openamber_write"
        json: |-
          root["ts"] = id(analytics_batch_timestamp);
          root["fw"] = ESPHOME_PROJECT_VERSION;
          // ... alle velden uit send_device_info t/m send_telemetry_4
        on_response:
          then:
            - if:
                condition:
                  lambda: return response->status_code != 200;
                then:
                  - logger.log: "Analytics POST mislukt"
    - script.execute: fetch_analytics_device_id
```

De lambda-inhoud van de huidige 13 `payload:`-blokken kan zo overgenomen worden. Om het overzichtelijk
te houden kunnen ze ook als één C++-helper `fill_analytics_json(JsonObject root)` in een header komen.

### Geheugen na de wijziging

| | MQTT (nu) | HTTP POST |
|---|---:|---:|
| Statisch, altijd | ≈ 22,3 KB | 0 (http_request zit er al in) |
| Runtime bij analytics aan | ≈ 9–10 KB **permanent** | 0 tussen requests |
| Piek tijdens verzenden | klein (per bericht) | ≈ 4–5 KB **tijdelijk**: rx-buffer 512 B + tx 1024 B + client-struct + JSON-body ~2–2,5 KB, na afloop vrijgegeven |
| Open sockets | 1 permanent | 0 tussen requests |

Netto winst: **≈ 22 KB voor iedereen**, en **≈ 31 KB** voor gebruikers met analytics aan. De tijdelijke
piek is kleiner dan de huidige permanente last en hangt niet vast in het geheugen, dus geen
blijvende fragmentatie.

### Aandachtspunten en risico's

- **Blokkerende loop**: `http_request` werkt op ESP-IDF synchroon in de main-loop. Dat doet
  `fetch_analytics_device_id` nu ook al elke 5 minuten. Beperken met `timeout: 3s` op `http_request`
  en met één request per interval in plaats van 13. Controllers met harde timing (compressor, pomp)
  gebruiken `millis()`-delta's (zie #311) en kunnen een vertraging van een paar honderd ms aan. Toch
  meten met `debug:` (loop-tijd) tijdens een POST.
- **Geen TLS**: MQTT praat nu ook plain TCP, dus HTTP is wat beveiliging betreft gelijkwaardig. Kies
  bewust **geen** HTTPS: met `MBEDTLS_SSL_IN/OUT_CONTENT_LEN=8192` kost een TLS-handshake tijdelijk
  ~20 KB+ en dat haalt de winst weer onderuit.
- **Retained-semantiek vervalt**: de server bewaart nu zelf de laatste staat per device. Daarvoor is
  een endpoint op `openamber.nl` nodig dat de JSON wegschrijft naar dezelfde opslag als de huidige
  MQTT-subscriber.
- **Grootte van de body**: ~130 velden ≈ 2–2,5 KB JSON. `buffer_size_tx: 1024` geldt voor de headers.
  De body schrijft esp_http_client los weg, dus dat past.
- **Backwards compatibility**: firmware die al in het veld draait blijft MQTT gebruiken. Laat de broker
  en subscriber dus draaien tot de meeste devices zijn bijgewerkt (de `fw`-key laat zien welke versie
  ze draaien).
- **Gemiste meting**: als een POST mislukt is die meting weg. Bij QoS 0 MQTT is dat nu ook zo, dus er
  verandert niets.

## Overwogen alternatieven

| Optie | Heap-winst | Serverwijziging | Complexiteit | Oordeel |
|---|---|---|---|---|
| **A. HTTP POST via `http_request`** (voorstel) | ≈ 22 KB + 10 KB | nieuw endpoint | laag, alleen YAML | **aanbevolen** |
| B. Eigen mini-MQTT-publisher (external component: socket → CONNECT → PUBLISH × n → DISCONNECT, QoS 0) | ≈ 22 KB + 10 KB | geen, broker blijft | middel: ~200 regels C++, zelf onderhouden | fallback als de server niet aangepast kan worden |
| C. UDP via `packet_transport`/`udp` | ≈ 22 KB + 10 KB | nieuwe UDP-listener | laag | afgeraden: geen bevestiging, payload > MTU moet je zelf splitsen, geen auth |
| D. MQTT houden en esp-mqtt kleiner configureren (kleinere stack/buffers via sdkconfig) | alleen ~3–4 KB runtime | geen | laag | lost de ~22 KB statische objecten niet op, want ESPHome maakt die altijd aan zodra `mqtt:` er staat |

## Implementatiestappen

1. **Server**: `POST /api/analytics/<mac>` (JSON, Bearer-token) en `POST /api/analytics/<mac>/eeprom_dump/<n>`
   toevoegen. Die schrijven naar dezelfde opslag als de MQTT-subscriber. Eerst live zetten.
2. **Firmware**:
   - `mqtt:` en `mqtt.enable` weghalen uit `src/openamber/core/openamber.yaml`.
   - De 13 `send_*`-scripts in `src/openamber/core/analytics_scripts.yaml` samenvoegen tot één
     `http_request.post` met `json:`.
   - `send_errors_realtime` en `dump_eeprom_parameters` omzetten naar `http_request`.
   - `timeout:` instellen op `http_request`.
   - De script-ids in `src/openamber/mock/mock_entities.yaml` en de uitzonderingslijst in
     `tests/test_entity_schema_sync.py` bijwerken als er ids verdwijnen.
3. **Meten** (voor en na, op de leejow- én de waveshare-build):
   - `xtensa-esp32-elf-size -A firmware.elf` → de `.dram0.bss`-sectie vergelijken.
   - Runtime: `system_heap_free`, `system_heap_max_block` en `system_heap_fragmentation` (die worden
     al meegestuurd als `yb`/`yd`/`yc`) na 24 uur met analytics aan en uit.
   - Loop-tijd tijdens een POST via `debug:`.
4. **Uitrol**: release-notes; broker parallel laten draaien tot het aandeel oude firmware
   verwaarloosbaar is.

## Open vragen

- Kan de backend op `openamber.nl` een HTTP-endpoint krijgen? (Zo niet, dan optie B.)
- Willen we een per-device token in plaats van het gedeelde `openamber_write`? Met HTTP kost dat geen
  extra moeite.
- Is het genoeg om de EEPROM-dump als één body te versturen (~1,5 KB per 125 registers, ~6 KB totaal),
  of willen we de chunks houden om de tijdelijke heap laag te houden? Advies: chunks houden.
