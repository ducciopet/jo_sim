# Protocollo esperimenti sul campo — spiaggia di sassi (A) e parco nazionale (B)

Obiettivo: dati per (1) misurare le etichette **D / PD / S** di v2 e (2) confrontare v2 con altri algoritmi (v1,
LV-DOT, Dynablox, M-detector, ...) sulle **stesse bag**, ai tre livelli dell'harness (D/PD/S, solo moto, D∪PD).

Principio: le persone (e ogni oggetto capace di muoversi) **partono dentro il FOV della camera**, così il sistema
può acquisirne la classe; poi si muovono liberamente attorno al robot.

---

## 1. Matrice degli esperimenti (12 + 4 controlli)

| | Robot fermo | Robot in movimento |
|---|---|---|
| **1 persona** | A1, B1 | A2, B2 |
| **3 persone** | A3, B3 | A4, B4 |
| **5 persone** | A5, B5 | A6, B6 |

Ogni esperimento: **2–3 minuti**, una esecuzione.

**Controlli (per ambiente, 1 minuto, nessuna persona)**: A0f / B0f robot fermo, A0m / B0m robot in movimento.
Misurano i falsi D/PD al minuto senza bisogno di GT (nel parco: vegetazione mossa dal vento).

Priorità se manca tempo: 1 e 3 persone in entrambi gli ambienti (A1–A4, B1–B4) + controlli.

---

## 2. Svolgimento di ogni esperimento

1. **Avvio (10 s, sempre uguale)**: tutte le persone **ferme dentro il FOV** della camera anteriore, a 3–5 m dal
   robot, distanziate. L'operatore dice ad alta voce l'ID dell'esperimento (es. "A3"), poi **salto tutti insieme**.
2. **Moto libero**: ognuno cammina e si ferma quando vuole, con tre regole:
   - ogni sosta dura **almeno 3–5 s**;
   - ognuno fa almeno **una sosta di lato o dietro** al robot;
   - **distanza minima ~1 m** dal robot.
3. **Fine**: tutti fermi 5 s dove si trovano, poi stop della registrazione.

**Robot in movimento**: sempre lo stesso percorso, lento (~0,3 m/s), con un tratto rettilineo e una curva o
un'inversione.

**In scena, in tutti gli esperimenti**: un oggetto statico (zaino, cassa o sedia) in una posizione misurata
rispetto alla partenza del robot → classe **S**, e prova per lo "scivolamento" dei track.

---

## 3. Ground truth senza mocap

1. **RTK sulle persone, se disponibile** (anche uno solo): negli esperimenti da 1 persona su di lei, negli altri a turno
   su una persona diversa. Posizione ~2 cm, nello stesso riferimento del GNSS del robot.
2. **Annotazione offline** dalle bag: posizioni a waypoint + istanti di partenza/sosta per ogni persona
   (formato `jo_detector_v2/eval/GT_SCHEMA.md`), guardando LiDAR grezzo e video, **senza** l'output di nessun metodo.
3. **Camera esterna su treppiede** (anche un telefono) che inquadra tutta l'area: riferimento per identità e istanti.
4. **Pettorine numerate** di colori diversi (P1…P5).

Etichette GT: **D** = in movimento (≥ 0,3 m/s); **PD** = persona ferma; **S** = oggetto non-persona fermo.

---

## 4. Cosa registrare (una bag per esperimento)

- Entrambe le camere: colore (compresso), **depth raw e allineata**, `camera_info`, `extrinsics/depth_to_color`.
- LiDAR (`/velodyne_packets` e `/velodyne_points`), IMU, GNSS/RTK (`/gnss`, `/rtcm`, `/nmea`), `/tf_static`,
  `/robot_description`.
- Nessun output dei detector: tutti i metodi si fanno girare dopo, sulle stesse bag.
- Nome: `<A|B><n>` (es. `A3`), riportato nel foglio di registro.
- **Spazio**: ~51 MB/s → 12 × 3 min + 4 × 1 min ≈ 40 min ≈ **120 GB** in totale.
- All'inizio, per ambiente: 60 s di robot fermo davanti a oggetti strutturati (calibrazione LiDAR–camera).

---

## 5. Foglio di registro

| Bag | Ora | Esperimento | Robot | Persone (pettorine) | Oggetto S e posizione | RTK su | Luce / vento | Note |
|---|---|---|---|---|---|---|---|---|

## 6. Checklist

- [ ] Spazio disco ≥ 150 GB; batterie (robot, laptop, camera esterna, RTK)
- [ ] Orologi sincronizzati (robot, PC, telefono della camera esterna)
- [ ] Rotella metrica, nastro per il segno di partenza del robot, oggetto S
- [ ] Pettorine numerate, treppiede + telefono
- [ ] Ricevitore RTK in più (se disponibile)
- [ ] Consenso delle persone riprese
- [ ] Per ogni ambiente: calibrazione (60 s) → controlli → esperimenti
- [ ] Dopo ogni bag: durata e topic completi
