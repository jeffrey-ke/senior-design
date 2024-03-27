# senior-design
## Requirements

1. Profiler can be launched from shore or in shallow waters of 1m deep water.
2. Profiler has an operational range of 2 kilometers from shore or 4 hours of continuous operation
3. While on the surface of the water, Profiler must know its location within 5 meters of accuracy.
4. While on the surface of the water, Profiler must have communication range of 1 kilometer.
5. While on the surface of the water, Profiler must be able to navigate to remotely commanded GPS waypoints with 5 meters of accuracy in swell heights of 1 meter.
6. Profiler completes at least 36 water column-profiles in 4 hours 
8. On leak or prohibtively low voltage, Profiler returns to the surface and back to the user.
9. Profiler weighs 10 kilograms and has a volume of 30 liters; profilers can be lifted by dive-hook and dropped into the water by 100% of typical marine researchers. 
10. While on the surface of the water, Profiler must transmit its location and status (battery voltage, internal pressure) to the user every 15 seconds.
11. After resurfacing from the dive, Profiler shall transmit recorded water-column parameters to user basestation with TCP-spec reliability.
12. Upon reaching GPS waypoints, Profiler must dive and descend to user-commanded depths of an accepted range of 0-100 meters.
13. While descending, Profiler must know its depth with 1 meters uncertainty. 
14. While descending, Profiler shall not drift laterally more than 20 meters. 
15. While descending, Profiler must know its surface coordinates with 5 meters of uncertainty.
16. Profiler can be remotely commanded to complete a sequence of dives at commanded GPS waypoints with TCP-spec reliability.
17. Profiler can be unpacked, booted up, and be ready for deployment in the water in less than 20 minutes.
18. Profiler navigates back to user after mission ends.
19. While descending, Profiler captures 2 water-samples at user-commanded and adaptively-determined depths.
20. Profiler shall autonomously analyze measured water-column temperatures to determine the depth of the top of the thermocline; profiler shall go-to that depth and capture a water-sample.
21. While descending, Profiler measures turbidity (+- 10 NTU) and temperature (+- 0.01 C) and depth (+- 0.01m) at 10 Hz
22. While descending, Profiler shall move at an user-commanded velocity of the accepted range of 0.1 m/s to 5 m/s.
23. User basestation shall transmit GPS coordinates to Profiler with TCP-spec reliability.
24. User basestation shall remotely command profilers with TCP-spec reliability to descend at a velocity within the accepted range and to descend to a depth within the accepted range.
25. User basestation shall display the current locations of the profilers every five minutes.
26. User basestation shall record water-column parameters of each dive transmitted by the profilers into separate data-files labeled by date, time, and coordinates.
27. User basestation shall graph water-column parameters of each dive.
28. Cluster of profilers shall be capable of following moving countours and extremums of scalar fields such as sediment plumes.

## Hardware

