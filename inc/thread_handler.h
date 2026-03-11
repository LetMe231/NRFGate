#ifndef THREAD_HANDLER_H
#define THREAD_HANDLER_H

/**
 * @brief UDP-Empfänger für OpenThread-Nodes initialisieren.
 *        Lauscht auf Port 5683 (ff03::1 Multicast) und
 *        loggt eingehende BMI088 Sensor-Pakete.
 *
 * @return 0 bei Erfolg, negativer errno-Wert bei Fehler.
 */
int thread_handler_init(void);

#endif /* THREAD_HANDLER_H */
