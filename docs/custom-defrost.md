# Defrost-regeling

OpenAmber kan zelf bepalen wanneer de buitenunit ontdooit. Alle defrost-instellingen staan onder
**Instellingen → Geavanceerd → Defrost**:

| Tegel | Inhoud | Zichtbaar |
| --- | --- | --- |
| Algemeen | Defrost controle; bij OpenAmber ook het minimale interval | altijd |
| Starttemperaturen unit | EEPROM-starttemperaturen van de buitenunit | bij Buitenunit |
| Einde defrost | EEPROM exit-temperatuur en maximale ontdooitijd | bij Buitenunit |
| Status | Status, draaitijd en laatste defrostduur | bij OpenAmber |

| Defrost controle | Wat er gebeurt |
| --- | --- |
| Buitenunit | De unit ontdooit zelf (standaard). OpenAmber houdt alleen statistiek bij. |
| OpenAmber | 3274 staat op master. OpenAmber start de defrost door werkmodus 4 te schrijven tijdens verwarmen met draaiende compressor. |

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

Er wordt alleen ontdooid tijdens verwarmen met draaiende compressor en Ta tot 7 °C, als Tp na het adaptieve interval
3 min onder −2 °C ligt. Het slot van 30 min van de buitenunit geldt altijd. Omdat het interval nooit langer is dan
120 min, is dit ook het vangnet: na 120 min draaitijd wordt er altijd ontdooid als Tp onder −2 °C ligt.

### Geen startlijn

Mitsubishi (technische handleiding SRK20-35ZSX-W, 17-SRK-T-228, hoofdstuk 17c) start een defrost als Tp een tijd
een vast verschil onder Ta ligt. Op de Amber werkt dat niet: met camerabeelden van de spoel (27/28 november 2026)
bleef Ta − Tp tussen 2,4 en 3,6 K, met én zonder rijp; bij de zwaarste rijp was het verschil juist het kleinst.
Ta − Tp zegt op de Amber dus niets over ijs, en die route is weggelaten.

### Interval

Na het adaptieve interval is een spoel onder −2 °C genoeg. Het interval past zich aan op de duur van de vorige
defrost die OpenAmber zelf startte, net als de Smart-stand van de buitenunit met 3341 = 0:

| Duur laatste defrost | Volgend interval |
| --- | --- |
| tot 3 min (weinig ijs) | +20 min, maximaal 120 min |
| 3 tot 5 min | gelijk |
| langer dan 5 min | −10 min |
| tot de maximale defrosttijd (3278) | terug naar het minimale interval |

De duur is de 2118-tijd van de geforceerde defrost. Die zegt veel over de hoeveelheid ijs: op 27/28 november 2026
(camerabeelden van de spoel) duurde een defrost zonder rijp 2:14–2:30, met lichte rijp 2:45–3:30 en met duidelijk witte
rijp 3:45–4:15. Langer dan 5 min betekent meer ijs dan toen bij de zwaarste rijp, dus dan was het interval te lang.

Het interval begint op het minimale interval (35 min, instelbaar van 30 tot 120 min) en is nooit korter dan 30 min. Zo ontdooit de regeling bij
vochtig weer vaker dan bij droog weer. Een defrost die de buitenunit zelf
startte, past het interval niet aan.

## Instellingen

Alleen wat het moment van ontdooien bepaalt is instelbaar: de defrost controle en het minimale interval. De overige waarden (maximale Ta, maximaal
interval en bevestigingstijd) staan vast in `controllers/constants.h`.

## Blokkades

- **Slot van de buitenunit.** Na een geforceerde defrost voert de unit een nieuwe pas uit na 30 min
  compressortijd. OpenAmber vraagt in die tijd niets aan (een handmatige aanvraag wordt dan geweigerd). De tijd komt uit
  de eigen draaitijdteller, zodat dit niet van de firmware van de buitenunit afhangt.
- **Handmatig forceren.** Onder **Defrost → Status** staat de knop "Forceren" (alleen op het display). Die werkt
  alleen tijdens verwarmen met draaiende compressor en buiten het slot van 30 min; anders wordt hij geweigerd (in de
  log). De aanvraag gaat bij de volgende gelegenheid uit als trigger "Handmatig" en vervalt als het verwarmen met de
  compressor stopt.
- **Bevestiging.** Zet de unit 2118 niet binnen 120 s aan, dan neemt OpenAmber werkmodus 4 terug en gaat de regeling
  in fout **TO10 - Defrost start timeout**, zonder nieuwe poging. Net als de andere timeoutfouten stopt dat de
  warmtepomp tot je de fout reset ("Reset systeem timeout fouten" of de resetknop bij de storingen). Er wordt geen
  nieuwe defrost aangevraagd zolang TO10 actief is. Stopt de compressor voordat de defrost begint, dan neemt OpenAmber
  werkmodus 4 terug zonder fout.
- **Tijdens de aanvraag** houden de verwarmings- en tapwaterregeling de compressor aan en schrijven ze geen werkmodus;
  zodra 2118 opkomt, gaan ze verder zoals bij elke defrost.

Na de defrost zet OpenAmber werkmodus 4 direct terug naar verwarmen. Anders zou de unit 30 min later opnieuw
ontdooien.

## Meten van een defrost

- **Defrostduur** (sensor "Laatste defrostduur"): hoe lang 2118 hoog was. Bij een defrost op eigen initiatief zet de
  buitenunit 2118 ongeveer 5 minuten vóór de echte defrost al aan (voorfase), dus dan zit die voorfase erin. Een
  geforceerde defrost heeft geen voorfase; alleen die past het interval aan.
- **Energie uit water:** warmte die de defrost uit het water haalde, in kJ: de integraal van het thermisch vermogen
  tijdens de defrost. Het thermisch vermogen wordt eens per minuut berekend, dus dit is een grove waarde.

Na elke defrost stuurt OpenAmber (met analytics aan) één bericht met deze waarden (het defrost-event), plus Ta en Tp bij de
start, de compressortijd sinds de vorige defrost, de defrost controle en de programmaversie van de buitenunit
(sleutels `dfa` t/m `dfg`, `dfk`, `dfn` en `dfo`; uitvoering `dfc` en smelttijd `dfd` worden niet meer gemeten en zijn
leeg).
