# TeseoProject

Questo programma permette la registrazione, salvataggio e visualizzazione dei dati, di percorsi eseguiti dall'utente, mediante una scheda che sfrutta il sistema GNSS.


Il programma è sviluppato e testato per la scheda **Nucleo F401-RE**, usando lo shield **X_NUCLEO_GNSS1A1**.

## Funzionamento programma

Le funzionalità principali disponibili per l'utente sono rappresentate mediante il seguente diagramma dei casi d'uso:

![image](https://user-images.githubusercontent.com/79313373/171050987-c12f9a9d-c01c-4784-9493-1c07f52a6127.png)

Funzionalità del programma:
1. Avvio percorso
2. Stampa lista dei percorsi salvati
3. Modifica impostazioni (unità di misura)

All'avvio del percorso il programma registra il punto nel quale avviene l'inizio del percorso, ovvero la posizione in cui si trova l'utente. Una volta avviato il percorso vengono visualizzate le statistiche, relative al percorso, aggiornate in tempo reale. Si può terminare il percorso, salvando il punto nel quale avviene la fine del percorso.

Una volta finito il percorso vengono stampati tutti i dati relativi ad esso e viene data la possibilità di salvare il percorso.

Dal menu principale si ha anche l'opzione di visualizzare i percorsi salvati. Una volta spenta la scheda i percorsi salvati vengono eliminati.

Si possono anche cambiare le unità di misura relative ai dati del percorso (es. da km a miglio, impostare il fuso orario dato l'orario UTC, ecc.).

### Immagini d'esempio

Menu principale:

![image](https://user-images.githubusercontent.com/79313373/171058304-2d8352fd-3a4d-4bd2-8a0a-e175bbb922bd.png)

