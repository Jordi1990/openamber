# Defrost-regeling

OpenAmber kan zelf bepalen wanneer de buitenunit ontdooit. Alle defrost-instellingen staan onder
**Instellingen → Geavanceerd → Defrost**:

| Tegel | Inhoud | Zichtbaar |
| --- | --- | --- |
| Algemeen | Defrost controle | altijd |
| Startlijn en interval | Verschil Ta − Tp, maximale Tp en minimaal interval | bij OpenAmber |
| Luchtvochtigheid | Bron, marge rijppunt, drempel ijsteller, actuele waarde en ijsteller | bij OpenAmber |
| Starttemperaturen unit | EEPROM-starttemperaturen van de buitenunit | bij Buitenunit |
| Einde defrost | EEPROM exit-temperatuur en maximale ontdooitijd | bij Buitenunit |
| Status | Status, draaitijd en laatste defrostduur | bij OpenAmber |

| Defrost controle | Wat er gebeurt |
| --- | --- |
| Buitenunit | De unit ontdooit zelf (standaard). OpenAmber houdt alleen statistiek bij. |
| OpenAmber | EEPROM 3274 staat op master. OpenAmber start de defrost door werkmodus 4 te schrijven. |

De stand wordt niet onthouden: na een herstart staat hij op Buitenunit en zet OpenAmber 3274 terug naar de
buitenunit (alleen als de teruggelezen waarde afwijkt). Je kunt altijd wisselen, ook vanuit Home Assistant (bijvoorbeeld naar Buitenunit als je niet thuis bent). Een defrost
die al loopt maakt eerst af; daarna zet OpenAmber werkmodus 4 terug naar verwarmen en schrijft 3274 opnieuw. Lukt een
geforceerde defrost twee keer niet, dan gaat waarschuwing OA01 aan en probeert OpenAmber het na 30 min opnieuw. De
controle blijft bij OpenAmber; alleen jij wisselt de stand.

## Wanneer wordt er ontdooid

Er wordt alleen ontdooid tijdens verwarmen met draaiende compressor en Ta tot 7 °C. Er zijn zes routes. De eerste
die geldt, start de defrost.

| Route | Wanneer | Wacht op vochtigheid |
| --- | --- | --- |
| Startlijn | Na het minimale interval: Tp ligt 3 min op of onder de startlijn | ja |
| IJsteller | Na het minimale interval: de ijsteller heeft de drempel bereikt (alleen met vochtigheidssensor) | nee |
| Interval | Na het adaptieve interval: Tp ligt 3 min onder −2 °C | ja |
| Pendelen | Na het adaptieve interval: ≥ 10 compressorstops, Ta ≤ 3 °C en Tp 3 min op of onder de maximale Tp | ja |
| Vangnet | Na 120 min draaitijd: Tp ligt 3 min onder −2 °C | nee |
| Diepe bevriezing | Tp ligt 3 min minstens 5 K onder de startlijn, zonder op een interval te wachten | nee |

De startlijn, het pendelen en de diepe bevriezing vragen daarnaast dat de compressor minstens 5 min onafgebroken
draait. Het slot van 30 min van de buitenunit geldt altijd.

### Startlijn

De startlijn volgt Mitsubishi (technische handleiding SRK20-35ZSX-W, 17-SRK-T-228, hoofdstuk 17c):
`Ta − benodigd verschil`, maar nooit hoger dan de **maximale Tp** (−5 °C).

| Ta | Benodigd verschil Ta − Tp | Startlijn Tp |
| --- | --- | --- |
| +5 °C | 5 K | −5 °C (maximale Tp) |
| 0 °C | 5 K | −5 °C (maximale Tp) |
| −5 °C | 4,7 K | −9,7 °C |
| −10 °C | 4,3 K | −14,3 °C |
| −15 °C en kouder | 4 K | −19 °C |

Mitsubishi gebruikt 7 K bij 0 °C. Bij de Amber ligt een schone spoel maar ±3 K onder Ta, en ijs maakt dat in
48 min nauwelijks groter (±0,4 K). Met 7 K werd de startlijn in november 2026 nooit gehaald. Bij 5 K ging hij in
die data nooit onterecht af, bij 4,5 K wel. Ontdooit de regeling te vroeg (korte defrosts), verhoog dan het
verschil; te laat (defrosts tot de maximale tijd), verlaag het.

### Interval

Na het adaptieve interval is een spoel onder −2 °C genoeg. Het interval past zich aan op de duur van de vorige
defrost die OpenAmber zelf startte, net als de Smart-stand van de buitenunit met 3341 = 0:

| Duur laatste defrost | Volgend interval |
| --- | --- |
| tot 3 min (weinig ijs) | +20 min, maximaal 120 min |
| 3 tot 5 min | gelijk |
| langer dan 5 min | −10 min |
| tot de maximale defrosttijd (3278) | terug naar het minimale interval |

Het interval begint op het minimale interval (35 min) en is nooit korter dan 30 min. Zo ontdooit de regeling bij
vochtig weer vaker dan bij droog weer, ook als de startlijn niet gehaald wordt. Een defrost die de buitenunit zelf
startte, past het interval niet aan.

### IJsteller

Met een vochtigheidssensor telt OpenAmber hoeveel ijs er groeit: het verschil tussen de dampdruk van de buitenlucht
en de dampdruk boven ijs bij Tp (Pa), opgeteld over de compressortijd sinds de laatste defrost (Pa·h). Ligt de spoel
boven het rijppunt, dan telt er niets bij. Bereikt de teller de **drempel ijsteller** (standaard 70 Pa·h), dan wordt
er na het minimale interval ontdooid. 0 zet de ijsteller uit.

Ter indicatie (november 2026, Ta −1 tot −3 °C): bij 90–95 % RV groeide de teller met 60–100 Pa per uur en lag er na
48 min zichtbaar ijs (uitvoering ±4 min). Bij 80 % RV was dat 25–50 Pa per uur en bleef de spoel na 48 min schoon
(uitvoering ±2 min). Met 70 Pa·h wordt het dan ±50 min bij mist en ±105 min bij 80 % RV.

### Pendelen, vangnet en diepe bevriezing

**Pendelende compressor.** De tweede route van Mitsubishi, voor weer waarin de compressor niet lang genoeg aan
blijft voor de startlijn.

**Vangnet.** Na 120 min draaitijd wordt er altijd ontdooid als Tp onder −2 °C ligt, ook als de vochtigheid zegt dat
de lucht droog is.

**Diepe bevriezing.** Ligt Tp ver onder de startlijn, dan is de spoel duidelijk bevroren. Er wordt dan ontdooid
zonder op het interval of de luchtvochtigheid te wachten.

## Instellingen

Alleen wat het moment van ontdooien bepaalt is instelbaar: de defrost controle, het minimale interval, het verschil
bij 0 °C en bij −15 °C, de maximale Tp, de marge rijppunt en de drempel ijsteller. De overige waarden (maximale Ta, vangnet,
retourblokkade, bevestigingstijd en maximaal uitstel droge lucht) staan vast in `controllers/constants.h`.

## Blokkades

- **Slot van de buitenunit.** Na een geforceerde defrost voert de unit een nieuwe pas uit na 30 min
  compressortijd. OpenAmber vraagt in die tijd niets aan (ook een handmatige aanvraag wacht). De tijd komt uit
  de eigen draaitijdteller, zodat dit niet van de firmware van de buitenunit afhangt.
- **Handmatig forceren.** Onder **Defrost → Status** staat de knop "Forceren" (alleen op het display). Die werkt
  alleen tijdens verwarmen met draaiende compressor. De aanvraag wacht op het slot van 30 min en vervalt als de
  compressor stopt of na 40 min.
- **Retourtemperatuur.** Onder 20 °C retourtemperatuur wordt gewacht, maximaal 15 min.
- **Bevestiging.** Zet de unit 2118 niet binnen 120 s aan, dan neemt OpenAmber werkmodus 4 terug en probeert het
  nog één keer.

Na de defrost zet OpenAmber werkmodus 4 direct terug naar verwarmen. Anders zou de unit 30 min later opnieuw
ontdooien.

## Optionele vochtigheidssensor

IJs groeit alleen als de spoel kouder is dan het rijppunt van de buitenlucht. Met een vochtigheidssensor rekent
OpenAmber dat rijppunt uit. Ligt Tp erboven, dan wacht de regeling, ook als de startlijn bereikt is: de lucht is
te droog om ijs te vormen. Het vangnet werkt altijd, ook zonder of met een verkeerde sensor.

De instellingen staan onder **Defrost → Luchtvochtigheid**:

| Instelling | Standaard | Betekenis |
| --- | --- | --- |
| Bron buitenvochtigheid | Extern | Alleen op displays met Bluetooth: een gekoppelde BLE-sensor of extern. |
| Marge rijppunt | 0 K | Tp moet zoveel onder het rijppunt liggen voordat er ijs telt. Hoger = later ontdooien in droge lucht. |
| Drempel ijsteller | 70 Pa·h | Ontdooien zodra er zoveel ijsaangroei geteld is (zie IJsteller). 0 = uit. |

Na het interval plus 60 min telt de vochtigheid niet meer mee. Dat beschermt tegen een verkeerde of verlopen sensor.

Op dezelfde pagina staan de actuele vochtigheid, het rijppunt en de ijsteller.

Er is geen aparte schakelaar: de controle werkt zodra er een geldige vochtigheid is. Zonder geldige waarde (geen
sensor, of geen meting van de BLE-sensor in de laatste 10 min) doet de vochtigheid niet mee en werkt de regeling zoals
zonder sensor.

### Bluetooth-sensor (Waveshare-displays)

1. Zet Bluetooth aan onder **Instellingen → Connectiviteit → Bluetooth** en koppel de sensor (SwitchBot, BTHome of
   Xiaomi met ATC-firmware). Er zijn drie plekken voor sensoren, samen met de ruimtesensor.
2. Kies de sensor bij **Defrost → Luchtvochtigheid → Bron buitenvochtigheid** (of in Home Assistant met de
   select "Luchtvochtigheid buiten bron").
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

## Meten van een defrost

Register 2118 is geen goede maat voor de defrostduur. Bij een defrost op eigen initiatief zet de buitenunit 2118
ongeveer 5 minuten vóór de echte defrost al aan (voorfase), terwijl de compressor dan nog gewoon verwarmt. Hoe lang die
voorfase duurt hangt af van de firmware, en de interne registers (5149 en hoger) verschillen ook per firmware.

OpenAmber meet daarom fysiek, op elke firmware:

- **Duur incl. voorfase:** hoe lang 2118 hoog was.
- **Defrostduur** (sensor "Laatste defrostduur"): vanaf het moment dat Tp minstens 3 K boven het laagste punt van de
  laatste minuut komt (heet gas op de spoel) tot 2118 weer laag is. Bij een geforceerde defrost zonder gedetecteerde
  stijging: de hele 2118-tijd.
- **Smelttijd:** hoe lang Tp tijdens de uitvoering tussen −1 en 3 °C bleef. Zolang er ijs smelt blijft de spoel rond
  0 °C hangen; dit is de beste maat voor de hoeveelheid ijs.
- **Energie uit water:** warmte die de defrost uit het water haalde: debiet × (Tui − Tuo), in kJ.

Alleen de defrostduur is een sensor. De andere waarden staan in het defrost-event hieronder.

Het interval past zich alleen aan op de uitvoeringsduur van een defrost die OpenAmber zelf startte.

Na elke defrost stuurt OpenAmber (met analytics aan) één bericht met deze waarden (het defrost-event), plus Ta, Tp, startlijn, vochtigheid
en rijppunt bij de start, de compressortijd en het aantal stops sinds de vorige defrost, de defrost controle
en de programmaversie van de buitenunit (sleutels `dfa` t/m `dfl`, `dfn` en `dfo`).
