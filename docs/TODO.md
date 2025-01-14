# TODO

- Le variabili di decisione dipendono soltanto dal numero di step di controllo e non da Fprime
- Levo i footstep dall'essere variabili di decisione

## Planner

- Pianifico un certo numero di step (non troppo grande)
- Un caso semplice puó essere quello di pianificare un solo step
- Supponiamo che siamo all'istante zero e che il prossimo timestamp é programmato per 0.89
- Programmo dove deve finire sto prossimo footstep, ma poi per calcolare la velocitá desiderata dello zmp difatti uso il moving constraint
- Cos'é il moving constraint?
  - Sono i midpoint del rettangolo che congiunge il piede di supporto allo step corrente al piede di support dello step successivo


### Ma quindi com'é la pipeline?

- Si inizia in double support
