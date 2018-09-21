 Resetter_TWI
 External Resetter with ATTiny85 for HomeCentral with atmega328. The Resetter checks whether the SDS-line of the atmega changes in a certain time. If not, eg the TWI hangs, the atmega is resettet by turning off and on the power with a relay.
 If the attiny hangs, it is resettet by the watchdog after signaling the issue to a raspberrypi managing the internet connection.
