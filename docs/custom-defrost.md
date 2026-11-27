# Defrost-regeling

OpenAmber kan zelf bepalen wanneer de buitenunit ontdooit. Alle defrost-instellingen staan onder
**Instellingen → Geavanceerd → Defrost**:

| Tegel | Inhoud | Zichtbaar |
| --- | --- | --- |
| Algemeen | Defrost controle | altijd |
| Startlijn en interval | Verschil Ta − Tp, maximale Tp en minimaal interval | bij OpenAmber |
| Starttemperaturen unit | EEPROM-starttemperaturen van de buitenunit | bij Buitenunit |
| Einde defrost | EEPROM exit-temperatuur en maximale ontdooitijd | bij Buitenunit |
| Status | Status, draaitijd en laatste defrostduur | bij OpenAmber |

| Defrost controle | Wat er gebeurt |
| --- | --- |
| Buitenunit | De unit ontdooit zelf (standaard). OpenAmber houdt alleen statistiek bij. |
| OpenAmber | 3274 staat op master. OpenAmber start de defrost door werkmodus 4 te schrijven. |

De stand wordt onthouden, ook na een herstart of update. OpenAmber zet 3274 één keer goed (alleen als de teruggelezen
waarde afwijkt): na het opstarten, na elke wissel van de stand en als de Modbus-verbinding met de buitenunit terugkomt.
Daarna wordt 3274 niet steeds opnieuw geschreven. Een Modbus-schrijfactie op 3274 wijzigt alleen het RAM van de
buitenunit, niet het EEPROM; na een stroomonderbreking van de buitenunit staat 3274 weer op de EEPROM-waarde tot
OpenAmber de verbinding terugziet en hem opnieuw schrijft. Je kunt altijd wisselen, ook vanuit Home Assistant
(bijvoorbeeld naar Buitenunit als je niet thuis bent). Een defrost
die al loopt maakt eerst af; daarna zet OpenAmber werkmodus 4 terug naar verwarmen en schrijft 3274 opnieuw. Start een
geforceerde defrost niet, dan stopt de regeling met fout TO10 (zie Blokkades). De controle blijft bij OpenAmber;
alleen jij wisselt de stand.

## Wanneer wordt er ontdooid

Er wordt alleen ontdooid tijdens verwarmen met draaiende compressor en Ta tot 7 °C. Er zijn twee routes, allebei
pas na het minimale interval. De eerste die geldt, start de defrost.

| Route | Wanneer |
| --- | --- |
| Startlijn | Na het minimale interval: Tp ligt 3 min op of onder de startlijn |
| Interval | Na het adaptieve interval (maximaal 120 min): Tp ligt 3 min onder −2 °C |

De startlijn vraagt daarnaast dat de compressor minstens 5 min onafgebroken draait. Het slot van 30 min van de
buitenunit geldt altijd. Omdat het interval nooit langer is dan 120 min, is de intervalroute ook het vangnet: na
120 min draaitijd wordt er altijd ontdooid als Tp onder −2 °C ligt.

### Startlijn

De startlijn volgt Mitsubishi (technische handleiding SRK20-35ZSX-W, 17-SRK-T-228, hoofdstuk 17c):
`Ta − verschil`, maar nooit hoger dan de **maximale Tp** (−5 °C). Het **verschil Ta − Tp** is standaard 5 K.
Mitsubishi laat het verschil richting −15 °C afnemen; OpenAmber gebruikt één verschil, voor de eenvoud.

| Ta | Startlijn Tp (verschil 5 K) |
| --- | --- |
| +5 °C | −5 °C (maximale Tp) |
| 0 °C | −5 °C (maximale Tp) |
| −5 °C | −10 °C |
| −10 °C | −15 °C |
| −15 °C | −20 °C |

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

Het interval begint op het minimale interval (35 min, instelbaar van 30 tot 120 min) en is nooit korter dan 30 min. Zo ontdooit de regeling bij
vochtig weer vaker dan bij droog weer, ook als de startlijn niet gehaald wordt. Een defrost die de buitenunit zelf
startte, past het interval niet aan.

## Instellingen

Alleen wat het moment van ontdooien bepaalt is instelbaar: de defrost controle, het minimale interval, het verschil
Ta − Tp en de maximale Tp. De overige waarden (maximale Ta, maximaal
interval, retourblokkade en bevestigingstijd) staan vast in `controllers/constants.h`.

## Blokkades

- **Slot van de buitenunit.** Na een geforceerde defrost voert de unit een nieuwe pas uit na 30 min
  compressortijd. OpenAmber vraagt in die tijd niets aan (een handmatige aanvraag wordt dan geweigerd). De tijd komt uit
  de eigen draaitijdteller, zodat dit niet van de firmware van de buitenunit afhangt.
- **Handmatig forceren.** Onder **Defrost → Status** staat de knop "Forceren" (alleen op het display). Die werkt
  alleen tijdens verwarmen met draaiende compressor en buiten het slot van 30 min; anders wordt hij geweigerd (in de
  log). De aanvraag gaat bij de volgende gelegenheid uit en vervalt als het verwarmen met de compressor stopt.
- **Retourtemperatuur.** Onder 20 °C retourtemperatuur wordt gewacht, maximaal 15 min.
- **Bevestiging.** Zet de unit 2118 niet binnen 120 s aan, dan neemt OpenAmber werkmodus 4 terug en gaat de regeling
  in fout **TO10 - Defrost start timeout**, zonder nieuwe poging. Net als de andere timeoutfouten stopt dat de
  warmtepomp tot je de fout reset ("Reset systeem timeout fouten" of de resetknop bij de storingen). Er wordt geen
  nieuwe defrost aangevraagd zolang TO10 actief is.

Na de defrost zet OpenAmber werkmodus 4 direct terug naar verwarmen. Anders zou de unit 30 min later opnieuw
ontdooien.

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

Na elke defrost stuurt OpenAmber (met analytics aan) één bericht met deze waarden (het defrost-event), plus Ta, Tp en startlijn bij
de start, de compressortijd sinds de vorige defrost, de defrost controle en de programmaversie van de buitenunit
(sleutels `dfa` t/m `dfh`, `dfk`, `dfn` en `dfo`).
