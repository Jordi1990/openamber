# Eigen defrost-regeling

OpenAmber kan zelf bepalen wanneer de buitenunit ontdooit. De instellingen staan onder
**Instellingen → Geavanceerd → Eigen defrost**.

| Regie | Wat er gebeurt |
| --- | --- |
| Buitenunit | De unit ontdooit zelf (standaard). OpenAmber houdt alleen statistiek bij. |
| Dry-run | Het algoritme hieronder draait en logt wanneer het zou ontdooien. De unit ontdooit nog zelf. |
| OpenAmber | EEPROM 3274 staat op master. OpenAmber start de defrost door werkmodus 4 te schrijven. |

## Wanneer wordt er ontdooid

Het algoritme volgt Mitsubishi (technische handleiding SRK20-35ZSX-W, 17-SRK-T-228, hoofdstuk 17c).
Een defrost start alleen als alles tegelijk geldt:

1. De compressor heeft sinds de vorige defrost minstens het **interval** gedraaid (start op 35 min).
2. De compressor draait minstens 5 min onafgebroken.
3. De spoeltemperatuur Tp ligt 3 min lang op of onder de **startlijn**.

De startlijn is `Ta − benodigd verschil`, maar nooit hoger dan de **maximale Tp** (−5 °C):

| Ta | Benodigd verschil Ta − Tp | Startlijn Tp |
| --- | --- | --- |
| +5 °C | 7 K | −5 °C (maximale Tp) |
| 0 °C | 7 K | −7 °C |
| −5 °C | 6 K | −11 °C |
| −10 °C | 5 K | −15 °C |
| −15 °C en kouder | 4 K | −19 °C |

Het verschil bij 0 °C (7 K), bij −15 °C (4 K) en de maximale Tp (−5 °C) zijn de waarden van Mitsubishi en
zijn instelbaar. De Amber is een grotere monoblock met een andere verdamper, dus controleer ze eerst in dry-run.
Ontdooit de regeling te vroeg (korte defrosts), verhoog dan het verschil; te laat (defrosts tot de maximale tijd),
verlaag het.

**Pendelende compressor.** Als de compressor sinds de vorige defrost minstens 10 keer gestopt is, Ta ≤ 3 °C is en
Tp 3 min op of onder de maximale Tp ligt, wordt er ook ontdooid. Dit is de tweede route van Mitsubishi, voor
weer waarin de compressor niet lang genoeg aan blijft voor de startlijn.

**Vangnet.** Na de maximale draaitijd (120 min) wordt er altijd ontdooid als Tp 3 min onder de vangnet-drempel
(−2 °C) ligt.

## Interval

Het interval past zich aan op de duur van de vorige defrost die OpenAmber zelf startte, net als de Smart-stand
van de buitenunit:

| Duur laatste defrost | Volgend interval |
| --- | --- |
| tot 3 min (weinig ijs) | +20 min, maximaal 120 min |
| 3 tot 5 min | gelijk |
| langer dan 5 min | −10 min |
| tot de maximale defrosttijd (3278) | terug naar het minimale interval |

Het minimale interval is instelbaar, maar nooit korter dan 30 min.

## Blokkades

- **Slot van de buitenunit.** Na een geforceerde defrost voert de unit een nieuwe pas uit na 30 min
  compressortijd. OpenAmber vraagt in die tijd niets aan (ook een handmatige aanvraag wacht). Bij firmware
  0x0110 komt de tijd uit register 5177, anders uit de eigen draaitijdteller.
- **Retourtemperatuur.** Onder de minimale retourtemperatuur wordt gewacht, maximaal de ingestelde wachttijd.

Na de defrost zet OpenAmber werkmodus 4 direct terug naar verwarmen. Anders zou de unit 30 min later opnieuw
ontdooien.

## Optionele vochtigheidssensor

IJs groeit alleen als de spoel kouder is dan het rijppunt van de buitenlucht. Met een vochtigheidssensor rekent
OpenAmber dat rijppunt uit. Ligt Tp erboven, dan wacht de regeling, ook als de startlijn bereikt is: de lucht is
te droog om ijs te vormen. Het vangnet werkt altijd, ook zonder of met een verkeerde sensor.

De instellingen staan onder **Eigen defrost → Luchtvochtigheid**:

| Instelling | Standaard | Betekenis |
| --- | --- | --- |
| Luchtvochtigheid gebruiken | uit | Zet de controle op het rijppunt aan. |
| Bron buitenvochtigheid | Extern | Alleen op displays met Bluetooth: een gekoppelde BLE-sensor of extern. |
| Marge rijppunt | 0 K | Tp moet zoveel onder het rijppunt liggen voordat er ijs telt. Hoger = later ontdooien in droge lucht. |
| Maximaal uitstel droge lucht | 60 min | Na het interval plus deze tijd telt de vochtigheid niet meer mee. 0 = onbeperkt. Beschermt tegen een verkeerde of verlopen sensor. |

Op dezelfde pagina staan de actuele vochtigheid en het rijppunt.

Zonder geldige waarde (geen meting, of geen meting van de BLE-sensor in de laatste 10 min) werkt de regeling zoals
zonder sensor.

### Bluetooth-sensor (Waveshare-displays)

1. Zet Bluetooth aan onder **Instellingen → Connectiviteit → Bluetooth** en koppel de sensor (SwitchBot, BTHome of
   Xiaomi met ATC-firmware). Er zijn drie plekken voor sensoren, samen met de ruimtesensor.
2. Kies de sensor bij **Eigen defrost → Luchtvochtigheid → Bron buitenvochtigheid** (of in Home Assistant met de
   select "Luchtvochtigheid buiten bron").
3. Zet **Luchtvochtigheid gebruiken** aan.

OpenAmber zet de vochtigheid van die sensor dan in `outdoor_humidity`. Bij het ontkoppelen of uitzetten van Bluetooth
valt de bron terug op Extern.

### Externe sensor

Laat de bron op **Extern** staan en publiceer de relatieve vochtigheid (%) in de sensor `outdoor_humidity`.
Bijvoorbeeld vanuit Home Assistant:

```yaml
sensor:
  - platform: homeassistant
    id: ha_outdoor_humidity
    entity_id: sensor.buiten_luchtvochtigheid
    on_value:
      - sensor.template.publish:
          id: outdoor_humidity
          state: !lambda 'return x;'
```

Of met een lokale I²C-sensor (SHT4x, SHT3x met filterkap of BME280), op dezelfde manier via `on_value`.

### Plaatsing

Plaats de sensor in de aanzuiglucht van de buitenunit, beschut tegen regen en zon, en niet in de uitblaas (die lucht
is afgekoeld en altijd bijna 100 %). Een BLE-sensor moet binnen bereik van het display blijven; met een metalen
omkasting of een muur ertussen kan dat lastig zijn.
