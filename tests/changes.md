Perfetto—ho passato al setaccio **test\_CRSF.c** insieme a **CRSF.c/.h** e **CRSF\_types.h**. Qui sotto trovi:

1. una stima onesta della copertura attuale,
2. i buchi rimasti (perché non arrivi già al 100%),
3. cosa aggiungere ai test (e, dove serve, micro-tweak al codice) per chiudere tutto,
4. un file `.md` pronto con “era → diventa” e spiegazioni.

---

# 1) Copertura attuale (stima)

* **API pubbliche coperte**: `CRSF_init`, `CRSF_buildFrame`, `CRSF_processFrame`, `CRSF_getStats`, `CRSF_resetStats`, e — quando abilitato — `CRSF_setTimestampCallback` / `CRSF_isFrameFresh`. I test di init/clear, null-safety, length/CRC, address validation, counting delle stats e roundtrip per (quasi) tutti i frame ci sono e sono robusti.    &#x20;
* **Errore/edge cases coperti**: wrong length, wrong CRC, tipo non supportato, indirizzo non valido, stats getter/reset & conteggi, direct command con CRC interno errato.     &#x20;
* **Freshness**: i test ci sono ma si compilano solo con `CRSF_ENABLE_FRESHNESS_CHECK=1`. Attenzione: oggi **il tuo header forza `CRSF_ENABLE_FRESHNESS_CHECK=0` se è definito `CRSF_CONFIG_TX`**, quindi in build “TX+RX” rischi di escluderli. &#x20;

**Numeri (ordine di grandezza)**
Con l’attuale suite e build “TX+RX” direi **\~94–97% line coverage** del codice effettivamente compilato e **\~90–93% branch coverage**. Il delta al 100% viene quasi tutto da rami particolari (vedi §2) e da una piccola rigidità dell’API (`recType` obbligatorio in `CRSF_processFrame`).

---

# 2) Cosa manca (punti scoperti)

1. **`CRSF_processFrame`: `recType` obbligatorio**
   Se lo passi `NULL` oggi andresti in write su puntatore nullo: nessun test lo prova (correttamente), ma resta un ramo non copribile finché l’API non lo rende opzionale.&#x20;

2. **`CRSF_buildFrame`: `frameLength` non viene validato**
   I test coprono `crsf == NULL` e `frame == NULL`, ma non possono coprire “`frameLength == NULL`” senza hard crash. Serve un micro-guard. &#x20;

3. **MAVLink Envelope – branch di “capping” (data\_size > 58)**
   In decode il payload viene troncato a 58 (`(payload[1] > 58 ? 58 : payload[1]) + 2U`), ma nei test usi al massimo 58, quindi il ramo “>58” non è esercitato. &#x20;

4. **Voltages – caso “values = 0”**
   Hai già i test “0 values” per RPM e Temperature, ma non per `VOLTAGES`. Aggiungendolo, copri anche il fallback “inserisci almeno una misura” simile agli altri. (I test “0 values” esistono per RPM/Temperature: es. `test_roundtrip_temperature_zero_values`, `test_roundtrip_rpm_zero_values` nel file.)&#x20;

5. **Freshness in build “TX+RX”**
   Per coprire i rami di freshness devi assicurarti che il macro rimanga a 1 (vedi tweak in §4.3). I test di freshness sono già scritti.&#x20;

---

# 3) Cosa aggiungere ai test (solo test)

Ti propongo di creare queste nuove funzioni di test (stesso stile/cmocka del file attuale):

* **MAVLink Envelope – capping >58**
  Costruisci un envelope con `data_size = 70`, poi verifica che lato RX `data_size == 58` e che la lunghezza frame corrisponda al cap. Copre il ramo di troncamento.&#x20;

* **Voltages – values = 0**
  `tx.Voltages.V[0] = 0;` e `values = 0` in `CRSF_buildFrame`. Attesi: lato RX `V[0] == 0` e length minima valida. (Simmetrico ai tuoi test 0-values per Temperature/RPM.)&#x20;

* **RC edges** (opzionale ma utile per branchs): vettore con estremi (es. 988 µs e 2012 µs) per verificare pack/unpack ai limiti, oltre ai valori nominali che già testi.

> Con queste 2–3 aggiunte mirate arrivi tipicamente al **\~98–99% linee** e \~**95% rami**, senza toccare il codice. Per il 100% pieno, vedi i micro-tweak sotto.

---

# 4) Micro-tweak al codice (per sbloccare 100%)

Qui sotto trovi un `.md` pronto con “era → diventa” e spiegazioni. Le tre modifiche sono **backward-compatible** e mirate:

* rendere opzionale `recType` in `CRSF_processFrame` (evita un ramo non testabile),
* validare `frameLength` in `CRSF_buildFrame`,
* rendere “override-abile” `CRSF_ENABLE_FRESHNESS_CHECK` così i test di freshness girano anche in build TX+RX.

---

## ✅ file: `coverage_100_percent_plan.md`

````markdown
# Piano copertura 100% – CRSF

## 1) `CRSF_processFrame`: rendere opzionale `recType`

**Era**
```c
// CRSF.c
CRSF_Status_t CRSF_processFrame(CRSF_t* crsf, const uint8_t* frame, CRSF_FrameType_t* recType) {
    if (!crsf || !frame) { return CRSF_ERROR_NULL_POINTER; }
    ...
    *recType = (CRSF_FrameType_t)frame[2];
    ...
}
````

**Diventa**

```c
// CRSF.c
CRSF_Status_t CRSF_processFrame(CRSF_t* crsf, const uint8_t* frame, CRSF_FrameType_t* recType) {
    if (!crsf || !frame) { return CRSF_ERROR_NULL_POINTER; }
    ...
    if (recType) { *recType = (CRSF_FrameType_t)frame[2]; }
    ...
}
```

**Perché**: abilita un test di robustezza con `recType == NULL` e chiude un ramo altrimenti non copribile (attuale funzione non fa il check).&#x20;

## 2) `CRSF_buildFrame`: null-guard su `frameLength`

**Era**

```c
// CRSF.c (inizio di CRSF_buildFrame)
if (!crsf || !frame) { return CRSF_ERROR_NULL_POINTER; }
```

**Diventa**

```c
// CRSF.c
if (!crsf || !frame || !frameLength) { return CRSF_ERROR_NULL_POINTER; }
```

**Perché**: i test già coprono i NULL per `crsf` e `frame`, ma senza questa guard non puoi aggiungere un test per `frameLength == NULL` (crash). Completa la simmetria con `CRSF_processFrame`. &#x20;

## 3) Freshness: rendere l’override possibile in build TX+RX

**Era**

```c
// CRSF.h
#ifdef CRSF_CONFIG_TX
#undef CRSF_ENABLE_FRESHNESS_CHECK
#define CRSF_ENABLE_FRESHNESS_CHECK 0
#endif
#ifndef CRSF_ENABLE_FRESHNESS_CHECK
#define CRSF_ENABLE_FRESHNESS_CHECK 1
#endif
```

**Diventa**

```c
// CRSF.h
#ifndef CRSF_ENABLE_FRESHNESS_CHECK
#  ifdef CRSF_CONFIG_TX
#    define CRSF_ENABLE_FRESHNESS_CHECK 0
#  else
#    define CRSF_ENABLE_FRESHNESS_CHECK 1
#  endif
#endif
```

**Perché**: permette di forzare `-DCRSF_ENABLE_FRESHNESS_CHECK=1` anche quando è definito `CRSF_CONFIG_TX`, così i test di freshness (già presenti) si compilano ed eseguono in build “TX+RX”.&#x20;

## 4) (Opzionale) Esporre helper per Direct Command

**Era**

```c
// CRSF.h (prototipi commentati)
 // CRSF_encodeDirectCommand(...);
 // CRSF_buildDirectCommandFrame(...);
```

**Diventa**

```c
// CRSF.h
CRSF_Status_t CRSF_encodeDirectCommand(...);
CRSF_Status_t CRSF_buildDirectCommandFrame(...);
```

**Perché**: semplifica i test sui comandi diretti ed evita boilerplate nei test (oggi costruiti “a mano”). (Vedi prototipi già presenti ma commentati).&#x20;

```

---

# 5) Come eseguire la suite per il 100%

Due strade equivalenti:

- **A. Un’unica build TX+RX** con `-DCRSF_ENABLE_FRESHNESS_CHECK=1` (dopo la modifica al macro di cui sopra).  
- **B. Matrix di build e merge coverage**:  
  1) `-DCRSF_CONFIG_RX` (freshness on),  
  2) `-DCRSF_CONFIG_TX` (freshness off),  
  poi **merge** dei report (`lcov -a run_rx.info -a run_tx.info -o total.info`), così copri anche rami condizionati dai `#if`.

> Nota: i test “valid lengths” già coprono la quasi totalità delle combinazioni di lunghezza per frame tipici; aggiungi il caso “MAVLink Envelope > 60” per confermare l’errore di type-length, oltre al capping lato decode (58). :contentReference[oaicite:27]{index=27} :contentReference[oaicite:28]{index=28}

---

Se vuoi, posso prepararti subito gli stub delle nuove funzioni di test (MAVLink capping, Voltages 0-values, RC edges) nello stesso stile di `test_CRSF.c`, così li incolli e vai.
```
