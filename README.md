# senior-design
## Component block diagram
![alt text](https://github.com/jeffrey-ke/senior-design/blob/main/documentation/Component%20block%20diagram.png?raw=true)

## Conops
![alt text](https://github.com/jeffrey-ke/senior-design/blob/main/documentation/multi-robot%20conops.png?raw=true)
## Requirements

1. Profiler can be launched from shore or in shallow waters of 1m deep water.
2. Profiler has an operational range of 1 kilometer from shore or 4 hours of continuous operation (Ask Spicer: how long are the dredging shifts? We need to recharge these bad boys.)
3. While on the surface of the water, Profiler must know its location within 5 meters of accuracy (based on GPS variance).
4. While on the surface of the water, Profiler must have communication range of 1 kilometer.
5. While on the surface of the water, Profiler must be able to navigate to remotely commanded GPS waypoints with 5 meters of accuracy.
6. (TBD) Profiler is capable of station-keeping in rough marine environments of sea-state X within X meters. (Ask Kirkwood what the seastate is in Monterey; Ask Neumann what the seastate is in Tahoe)
7. (TBD) Profiler completes X water column-profiles in 4 hours (Ask Neumann + Kirkwood about how many they want)
8. On leak or prohibtively low voltage, Profiler returns to the surface and back to the user.
9. (TBD) Profiler weighs X kilograms and has a volume of X liters; profilers can be lifted by hand and dropped into the water by 100% of typical marine researchers. 
10. (TBD) While on the surface of the water, Profiler must transmit its location and status (battery voltage, internal pressure) to the user every X minutes. ****(Ask Spicer/Neumann/Kitts how often he wants updates from the profiler; and how often he wants to send new waypoints to the profiler)****
11. Upon reaching GPS waypoints, Profiler must dive and descend to a maximum depth of 100 meters.
12. (TBD) While descending, Profiler must know its depth with X meters uncertainty. **(Ask Spicer/Thomas/Neumann: How tall are these plumes? If it's in the order of magnitude of 10s of meters, then +- 1m accuracy should be fine)**
13. (TBD) While descending, Profiler descends vertically with less than X meters of lateral drifting. **(Ask Neumann/Thomas: How wide are the columns of water/plumes? If they're very narrow, then the profiler should have very little lateral drift; we can probably address this with proper ballasting i.e. the profiler is perfectly vertical.)**
14. (TBD) While descending, Profiler must know its surface coordinates with X meters of uncertainty. **(Plays into: how wide are the water columns? Ask Kirkwood if lateral drift is a big problem; we put this so that the profiler is relatively sure where in the water it read the data)**
15. Profiler can be remotely commanded to complete a sequence of dives at commanded GPS waypoints with TCP-spec reliability. **(Pixhawk??)**
16. (TBD) Profiler can be unpacked, booted up, and be ready for deployment in the water in less than X (20) minutes.
17. Profiler returns to user after mission ends.
18. (TBD) While descending, Profiler captures X water-samples at user-commanded depths with less than X meters of error. **(Ask Neumann and Kirkwood and Thomas)**
19. (TBD) While descending, Profiler measures turbidity and temperature with X% accuracy and X% resolution every X seconds. **(Ask Kirkwood or Spicer what level of resolution is typical to measuring turbidity for sediment plume tracking; also ask Kirkwood whether we should build a turbidity sensor, and what hardware he reccomends)**
20. (Extra credit:) Cluster of profilers capable of following a moving plume of sediment or other scalar readings.

## Hardware

