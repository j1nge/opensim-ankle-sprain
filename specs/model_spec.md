# Ankle Sprain Model — Spec (v0)

## Bodies & DOFs
- Tibia, Talus, Calcaneus (+ optional Forefoot hinge)
- Joints: Tibio-talar (PF/DF), Subtalar (Inv/Ev), (optional) Midfoot hinge

## Muscle-equivalents (tensile only)
- Peroneals (resist inversion)
- Tibialis anterior / posterior
- Gastro-soleus
- Params: stiffness k, dissipation c (scales TBD)

## ROM / Failure (TEMP placeholders → confirm later)
- TT: +30° PF, −20° DF  *(TODO: confirm)*
- ST: +35° inv, −15° ev *(TODO: confirm)*
- Failure: exceeding ROM (and later ligament strain > threshold)

## GRF Impulse
- Direction: sweep every 25° about axes *(grid TBD)*
- Peak: ~2×BW at t_peak ∈ {40,50,60,70,80,90,100} ms
- Residual: {0.25, 0.5, 0.75, 1.0}×BW to ~200 ms
- Application point: forefoot region (met heads)

## Outputs
- Peak TT/ST angles & velocities
- ROM exceed? which joint? time to failure
- Trace snippets for failing trials (TBD)
