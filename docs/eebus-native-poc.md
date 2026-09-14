# Native EEBUS op de ESP32 — PoC-onderzoeks- & implementatieblauwdruk

Doel: OpenAmber (ESPHome-firmware op ESP32(-S3/PSRAM) voor de Itho Amber warmtepomp)
laat zich **native, via EEBus** aansturen door **evcc** — zodat evcc de warmtepomp
kan boosten/dimmen op basis van PV-overschot, dynamische tarieven, etc.

Document vervangt geen handleiding; het is de blauwdruk waarmee de native PoC
stapsgewijs kan worden gebouwd en gevalideerd tegen een echte evcc-installatie.

---

## 1. Status & scope

- **Vastgesteld uit evcc-broncode** (blauwdruk is protocol-nauwkeurig): evcc's
  `eebus-ohpcf` driver (`charger/eebus-ohpcf.go`) beschrijft precies welke EEBus
  use-cases/function-blocks een **bestuurbare warmtepomp** moet implementeren.
- **Niet in deze sessie gebouwd/gevalideerd**: volledige SHIP/SPINE-stack is een
  grote, meerweken-effort opdracht en vereist hardware + echte evcc + EEBus-schema-
  tooling (zie §7). Deze blauwdruk is de basis om dat te doen.
- **Keuze**: native EEBUs op het device (alles-on-device), als PoC; niet de
  gateway-brug.

---

## 2. Uitgenomen protocol-contract (uit `evcc/charger/eebus-ohpcf.go`)

evcc speelt de rol van **CEM (Customer Energy Management)**, **Monitoring
Appliance (MA)** en **Energy Guard (EG)** *client*. De warmtepomp (OpenAmber) is
het **apparaat** en moet de corresponderende *server* use-cases implementeren.
(eebus-go usecases: `github.com/enbility/eebus-go/usecases`.)

| evcc-interface | EEBus use-case | Rol apparaat | Data | OpenAmber-koppeling |
|---|---|---|---|---|
| `api.Charger` (on/off) | **CEM/OHPCF** `usecases/cem/ohpcf` | compressor-entity | `RequestedPowerEstimate` (W, opt. opname), `RequestedPowerMax`, `ConsumptionIsStoppable/Pausable`, `ConsumptionStartTime`, `MinimalRunDuration`, `MinimalPauseDuration`, `ConsumptionState` (Available/Paused/Running/Scheduled); commando's `SchedulePowerConsumptionProcess`, `Resume...`, `Abort...`, `Pause...` | `heat_demand_active_sensor` / DHW-demand; blok/boost-logica |
| `api.Meter` (`CurrentPower`) | **MA/MPC** `usecases/ma/mpc` | levert live vermogen (`Power`, W) | geïntegreerde meter | `power_compressor` (+ backup heater + pompen) |
| `api.Battery` (`Soc`) | **MA/MDT** `usecases/ma/mdt` | levert tapwatertemperatuur | `Temperature` °C | `dhw_temperature_tw_sensor` |
| `api.Dimmer` (`Dim`) | **EG/LPC** `usecases/eg/lpc` | ontvangt `WriteConsumptionLimit{Value,IsActive}` (0 W safelimit) | §14a/LPC-limiet | SG-Ready blok/dim → compressor uit |

Belangrijk uit de driver:
- OHPCF is **on/off** (niet moduleerbaar): enable=schedule/resume, disable=pause/abort
  van de "optionele opname". `MaxCurrent` wordt genegeerd.
- De **optionele opname** (`RequestedPowerEstimate/Max`) is het *bovenop de
  basisvraag* schakelbare deel. OpenAmber bepaalt zelf hoeveel daarvan echt
  draait; evcc mag alleen een momentopname schakelen (binnen `MinimalRunDuration`/
  `MinimalPauseDuration`).
- `Dim()` schrijft bij dimmen een **0 W actieve limiet**; de limiet is een "veilige
  on-hold", geen absolute regeling.

---

## 3. Systeemarchitectuur

We voegen een **ESPHome external component** toe, analoog aan de bestaande
`openamber_component`, die een EEBus-device-stack opstart. Drie lagen:

1. **Kern (device-stack)** — nieuw, los in `src/openamber/eebus/`:
   - Certificate & keys (EC P-256, self-signed X.509) + persistence in NVS.
   - **SHIP**-transport: mDNS-advertentie (`_ship._tcp`), lokale TCP listener,
     TLS (mbedTLS), handshake/pairing, SHIP_IDENT/SKI, warm-up.
   - **SPINE**-core: datagram framing, node/device-management, binding/control,
     function-blocks & data-classes.
2. **Use-cases-laag** — implementeert de server-rollen uit §2 (OHPCF, MPC, MDT, LPC)
   en vertaalt EEBus-data naar een neutraal C++-object (OperationStress/mode,
   gemeten W, Tw).
3. **Integratielaag** — koppelt aan bestaande OpenAmber-entiteiten en controllers
   (`id(...)`), met hard safety-voorrang.

```
+----------------------------+
| evcc (CEM/MA/EG client)    |   lokale SHIP over TLS + SPINE
+-------------+--------------+
              | TCP/44328(mDNS _ship._tcp) TLS, SKI-pairing
+-------------v--------------+
| openamber/eebus component  |
|  - eebus_component         |  ESPHome component (devicestack lifecycle)
|  - ship/                   |  SHIP + mbedTLS listener
|  - spine/                  |  SPINE core (framing, control, FBs)
|  - usecases/               |  eg/ohpcf, ma/mpc, ma/mdt, eg/lpc (server)
|  - mapping/safety          |  -> id(...) controllers
+-------------+--------------+
              | modbus RS485
+-------------v--------------+
| Itho Amber (binnen/buiten) |
+---------------------------+
```

---

## 4. OpenAmber-mapping & veiligheid

| EEBus-events | Naar OpenAmber |
|---|---|
| OHPCF `Schedule/Resume` (enable) | laat bestaande SG-Ready/vraaglogica dienen; houd basisvraag; boost indien via SG-Ready boost |
| OHPCF `Abort/Pause` (disable) | bestaande stop/demand-route (geen geforceerde harde stop die veiligheid schendt) |
| LPC `WriteConsumptionLimit{0W,active}` (dim) | `sg_ready_block_mode_active_sensor`-route: compressor-vraag onderdrukken (zoals bestaand SG-Ready blok) |
| MDT `Temperature` | `dhw_temperature_tw_sensor` |
| MPC `Power` | `power_compressor` (+ backup heater + pompen) |

**Hard safety-voorrang (niet-onderhandelbaar):**
- Vorstbescherming (stages 1/2), `error_active`, minimum aan/uit-tijden van
  compressor en pompflowveiligheid **blijven te allen tijde voorrang houden**.
- EEBus-wensen **projecteren op bestaande demand/setpoint-logica**, nooit direct
  modbus-knoppen overrulen. Dimmen mag nooit de vorstbescherming of de minimale
  pompflow doorbreken.
- De OHPCF "optionele opname" is een **advies-schakelaar** binnen
  `MinimalPauseDuration`/`MinimalRunDuration`; de HP houdt zelf controle.
- Failsafe: bij SHIP/TLS-verlies of verlopen `failsafeDuration` terug naar
  "Normaal" (basisregeling), zodat de HP nooit in een geblokkeerde staat hangt.

---

## 5. Concrete component-indeling

> Let op: ESPHome compileert automatisch alleen `.cpp`/`.h` **in de top-level
> map** van een external component; submappen (`use cases/`, `ship/`, `spine/`)
> worden NIET meegecompileerd. Alle bronnen liggen daarom flat in `eebus/`.

```
src/openamber/eebus/
  __init__.py              # componentregistratie + configschema + bridge-lambdas
  eebus_component.h/.cpp   # lifecycle + bridge + MPC/MDT-refresh + failsafe-hook
  cert.h/.cpp              # gefixeerd/embedded EC-P256 X.509 + SKI + NVS
  nvs_store.h/.cpp         # NVS-wrapper
  ship_listener.h/.cpp     # FreeRTOS accept-task + mbedTLS TLS-serverconfig
  mdns.h/.cpp              # _ship._tcp-advertentie (TXT TODO)
  eebus_ohpcf.h/.cpp       # OHPCF server-state machine (opt. opname)
  eebus_lpc.h/.cpp         # LPC server (dim/§14a)
  eebus_measurements.h     # MPC (vermogen) + MDT (tapwatertemp)
```

Config-oppervlak (in `openamber-waveshare-display.yaml` o.i.d.):
```yaml
# voorbeeld-blok (uit te werken in __init__.py)
eebus:
  device_sku: "OpenAmber-ESP32"
  brand: "OpenAmber"
  model: "Amber"
  mqtt: false            # device-rol verbindt via SHIP, niet SEPC-cloud
  failsafe_duration: 2h
```

---

## 6. Randvoorwaarden ESP32 / mbedTLS / geheugen

- Gebruik de **ESP32-S3/PSRAM-variant** (`wifi: use_psram`, zie
  `openamber-waveshare-display.yaml`) — SPINE-schema + TLS is groot.
- mbedTLS config uitbreiden (EC keys, X.509 parse/sign, TLS 1.3 of 1.2 voldoende
  voor SHIP). Controleer free-PSRAM (`system_psram_free` sensor).
- Persist cert/keys + paired-SKI's in **NVS/preferences** zodat re-pairing niet
  nodig is na reboot.
- mDNS: ESP-IDF mDNS adv `_ship._tcp` met TXT (id, path, port, v, si, c, fn...).

---

## 7. Incrementele test- & validatiestrategie

1. **Off-device protocoltoets**: gebruik eebus-go's `mtools` / `mira` of een
   referentie-CEM om het device te laten koppelen en OHPCF/MPC/MDT/LPC te oefenen
   VOORDAT je het op hardware draait.
2. **SKI-pairing**: OpenAmber toont zijn SKI (uit X.509-subject/SubjectKeyId).
   evcc config `charger: type: eebus-ohpcf, ski: <ski>, ip: <ip>`.
3. Verifieer data-updates die evcc verwacht: `RequestedPowerEstimate/Max`,
   `ConsumptionState`-overgangen, `Power` (MPC), `Temperature` (MDT), en dat
   `Dim()` de 0 W-limiet schrijft.
4. Daarna gedrag: evcc in `pv`-modus → boost bij overschot, dim bij §14a; check
   dat OpenAmber-safety (frost, pompprotectie) EVCC-dim/-stop overschrijft.

---

## 8. Risico's & realistische tijdlijn

- Geen volwassen ESP-IDF SHIP/SPINE-bibliotheek; alles self-built. Hoog risico en
  veelvuldig afstemmen op de SPINE-schema-constanten van `spine-go`.
- Beslag op flash/PSRAM; LVGL-UI (Waveshare) concurreert.
- Pairing/cert-beheer is foutgevoelig; re-pairing na cert-wijziging.
- Schatting PoC-niveau: weken werk op de betrokken paden (gerelateerde
  `eg/ohpcf` vs `cem/ohpcf` rol-asymmetrie exact uitkristalliseren via eebus-go).

**Alternatief met lagere inspanning (bewust open gelaten):** dezelfde use-cases
implementeren in een **off-device gateway** (eebus-go container) die OpenAmber via
bestaande MQTT/HTTP-API aanstuurt — minder risico, zelfde evcc-beleving. Zie
onderzoek in chat.

---

## 9. Implementatiestatus (scaffold)

**Status: compileert + linkt tot geldig ESP32-image** (gevalideerd met `eshome
compile` op een minimale config die `eebus:` gebruikt). Alleen actief zodra
`eebus:` wordt toegevoegd; bestaande builds worden niet geraakt.

**Geïmplementeerd:**
- `__init__.py` — componentregistratie + configschema (`sku`/`brand`/`model`/`failsafe_duration` + OpenAmber-bridge lambdas `read_power`, `read_dhw_temp`, `apply_optional(value)`, `apply_dim(value)`, en de uitgebreide `apply_limit(active, limit_w)`).
- `cert.h/.cpp` — embedded EC P-256 self-signed pair + SKI, NVS-persistentie.
- `nvs_store.h/.cpp` — NVS-wrapper.
- `mdns.h/.cpp` — `_ship._tcp`-advertentie met SHIP-TXT-set (`txtvers=1`, `id`, `path=/ship/`, `ski`, `register=false`) via ESP-IDF mdns.
- `ship_listener.h/.cpp` — FreeRTOS TLS-server taak met mbedTLS, WebSocket accept, en een draadveilige `queue_outbound_frame()` wachtrij om notificaties asynchroon over de actieve verbinding te zenden.
- `eebus_component.h/.cpp` — lifecycle + bridge + MPC/MDT-refresh + change callbacks naar SpineNode + failsafe watchdog timer.
- `eebus_ohpcf.h/.cpp` — OHPCF server-state machine (AVAILABLE/SCHEDULED/RUNNING/PAUSED) met realistische default vermogensvraag (1500 W / 3000 W max) voor evcc zonne-surplus scheduling.
- `eebus_lpc.h/.cpp` — LPC-server (WriteConsumptionLimit / §14a dimmen) met ondersteuning voor zowel binaire dim als traploze/meertraps Watt-limieten (`LimitApplier`).
- `eebus_measurements.h` — MPC (actueel vermogen) + MDT (tapwatertemperatuur) sensor-integratie.
- `eebus_spine.h/.cpp` — SPINE 1.3.0 JSON datagram engine met recursieve JSON-parser (accolade-dieptetelling) om geneste structuren van evcc foutloos te parsen.
- `eebus_node.h/.cpp` — SPINE device-node: lokale compressor/cem-adressen, discovery/binding responses, heartbeat ACK, en proactieve `NOTIFY` datagrammen bij status- en meetwaardewijzigingen.
- `eebus_websocket.h/.cpp` — RFC 6455 WebSocket-server (Sec-WebSocket-Accept + frame encode/decode).
- `eebus_ship.h/.cpp` — SHIP framing en JSON-transformatie.

**Validatie in deze repo:** Volledige `openamber-waveshare-display.yaml` build compileert en linkt foutloos (`esphome compile`).

---

## 10. Protocol-annotaties — moderne SHIP/SPINE (uit ship-go/spine-go/eebus-go)

Gegrond in de referentie-repo's die evcc pin (`eebus-go v0.7`, `ship-go v0.6`,
`spine-go v0.7`). Deze sectie is de ground-truth voor een implementatie die
interopereert met evcc; corrigeer hier n.a.v. live evcc-testen.

### Transportlaag (SHIP)
- Discovery: mDNS `_ship._tcp`, instance/voor `id`, TXT met de SHIP-service-set.
- Transport: **WebSocket (RFC 6455)** over **TLS** op het mDNS-geadverteerde
  poortpad (`path`), met `specificationVersion` in de SHIP-connectie.
- **Vertrouwen/pairing:**
  - Klassiek: certificaat-**SKI** / fingerprint-verificatie; evcc-pairing via
    `ski:` + `ip:` config.
  - Nieuw (AddCu). `_ship._tcp` TXT-record-set voor pairing-service
    (`ShipPairingTXT`), o.a.: `txtvers=1`, `parType=fpSha256`, `forId`/`forPar`
    (eigen SHIP-ID + fpSHA256 van cert), `trustId`/`trustPar`, `trustCurve=secp256r1`,
    `type=addCu`, `trustNonce`, `alg=hmacSha256`, `digest`. Onderliggend SRP/hmac-asymmetrie.
- Certificaat: EC P-256 selfsigned; **SKI = subject key identifier (hex)** van
  het eigen cert; SKI normalisatie (b.v. `util.NormalizeSKI`, colon-gescheiden
  kleinkapitalen).

### Berichtlaag (SPINE)
SPINE-berichten zijn **JSON-datagrams**:
```json
{
  "datagram": {
    "header": {
      "specificationVersion": "1.3.0",
      "addressSource":      {"device": 1, "entity": 1, "feature": 1},
      "addressDestination": {"device": 2, "entity": 1, "feature": 1},
      "msgCounter": 1,
      "msgCounterReference": 0,
      "cmdClassifier": "write"
    },
    "payload": { "cmd": [ { "<dataClassName>": { ... } } ] }
  }
}
```
- `cmdClassifier`-waarden: `call`, `result`, `reply`, `write`, `read`,
  `notify`, `error`.
- Feature-address: `device` / `entity` / `feature` ints; controle-berichten
  (`nodeManagement...`) hebben lege feature of specifieke zb.
- Dataclasses (voor deze PoC benodigd):
  - Node mgmt: `nodeManagementBindingData`, `nodeManagementDetailedDiscoveryData`,
    `nodeManagementUseCaseData`, `nodeManagementPermittedConnectionsData`,
    `nodeManagementKeyValueDescriptionListData`.
  - Device: `deviceClassificationManufacturerDataType`,
    `deviceClassificationDeviceDataType` (`deviceCategory=heatPump`),
    `deviceConfigurationKeyValueData`, `deviceInformationDescriptionData`,
    `deviceInformationDetailData`.
  - Measurement: `measurementListData` (MPC `power`, MDT `temperature`).
  - OHPCF (device = **Compressor** entity, `smartEnergyManagementPs` feature):
    opt. opname via power forecast / device operations; `requestedPower*`,
    `operationState`/processstate (Available/Scheduled/Running/Paused),
    `minimalRunDuration`, `minimalPauseDuration`, `notStartBeforeTime`,
    commands `schedule`/`resume`/`pause`/`abort`.
  - LPC (EnergyGuard): WriteConsumptionLimit `consumptionLimit` (`value`, `isActive`).
- Bindings: device `call`(:`node_management_binding`?) schrijft
  `nodeManagementBindingData` om CEM-features te binden; vervolgens `reply`.
- `msgCounter` moet oplopen per bericht; bij `request` `ackRequest` etc.

### Gebruiksscenario/rollen (device-kant)
- `UseCaseActorTypeCEM` ↔ device `UseCaseActorTypeCompressor`
  (`OptimizationOfSelfConsumptionByHeatPumpCompressorFlexibility`, scenarios 1+2).
- Monitoring Appliance: MPC (power) + MDT (tapwatertemp).
- Energy Guard: LPC (consumptielimiet/dim).
Zie evcc `charger/eebus-ohpcf.go` voor precies welke use-cases/data evcc leest.

### Teststrategie tegen evcc
1. Laat evcc `usi/detect` of `evcc device` de OpenAmber-SHIP-pairing vinden via
   `_ship._tcp`; noteer de SKI.
2. evcc.yaml `chargers: - type: eebus-ohpcf, ski: <SKI>, ip: <ip>`.
3. Volg logvolgorde: TLS/WS-connect → pairing/trust → SPINE node-mgmt/binding →
   OHPCF/MPC/MDT/LPC data – vaststellen welke stap als eerste faalt en daarop
   itereren (`spine-go` + `spine-go` testdata als referentie).


