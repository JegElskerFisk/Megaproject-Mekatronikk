# scene

Publiserer kollisjonsobjekter (kuber og bord) til MoveIt-planleggingsscenen, og kringkaster posisjonene deres via TF. Pakken er nyttig for å simulere interaksjon med flere objekter i RViz.

---

## Innhold

- [Oversikt](#oversikt)
- [Kjøring](#kjøring)
- [Noder](#noder)
- [Systeminteraksjon](#systeminteraksjon)

---

## Oversikt

Pakken består av to Python-noder:

- `interactive_scene.py`: Publiserer tre fargede kuber til MoveIt og sender posisjonene deres kontinuerlig med TF. Lytter også etter interaktiv flytting i RViz.
- `table_scene.py`: Publiserer et statisk bord som kollisjonsobjekt én gang.

Begge noder startes med `scene.launch.py`.

---

## Kjøring

```bash
ros2 launch scene scene.launch.py
```

> Forutsetter at MoveIt er aktivert og at `apply_planning_scene`-tjenesten er tilgjengelig.

---

## Noder

| Node               | Forklaring                                      |
|--------------------|--------------------------------------------------|
| `interactive_scene`| Tre kuber, TF-publisering og sceneoppdatering   |
| `table_scene`      | Statisk bord legges til i scenen én gang        |

---

## Systeminteraksjon

- **Publisering**: `/apply_planning_scene`
- **TF**: `cube1`, `cube2`, `cube3` kringkastes relativt til `base`
- **Subscribing**: `/monitored_planning_scene` for å oppdatere kubenes posisjon

