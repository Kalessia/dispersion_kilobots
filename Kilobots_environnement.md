# Kilobots environnement


- **Kilolib** : utiliser celle sur GitHub à l’adresse https://github.com/acornejo/kilolib  

- **Flashage** : utiliser les documents fournis par Yoones, disponibles dans le répertoire *flashage*.  
	>		controller.hex, pour flasher la douche  
    >		bootloader.hex, pour flasher les kilobots  
	La procédure de flashage est disponible sur le site https://diode.group.shef.ac.uk/kilobots/index.php/Kilobot_Firmware, et résumée dans le fichier *procedure_flashage* dans le répertoire *flashage*.
	
- **KiloGUI** : nous avons installé celle sur GirHub à l’adresse https://github.com/acornejo/kilogui/releases (*Source code*). Nous avons installé le *FTDI driver* via la librairie *libftdi1*, pour utiliser la kiloGUI en mode FTDI. 
	>		Procédure pour lancer la kiloGUI :
	>		- sur terminal, utiliser la commande « sudo kilogui »
	NB. Ne surtout pas utiliser la commande « sudo kilogui &disown ». Si c’est le cas, il faut tuer les processus *sudo* et *sudo kilogui*. 
	>		Procédure pour tuer les deux processus. Sur terminal, utiliser les commandes :
	>		- ps ax|grep kilogui (on obtient les codes des processus actifs)
	>		- kill -q (code sudo) (code sudo kilogui)   
	NB. Nous n'avons pas vérifié que la kiloGUI installée à partir du code source fonctionne mieux que la kiloGUI installée par fichier executable (Ubuntu, Win, Mac). 
	
- **avr-gcc** : n'importe quelle version semble bien fonctionner. Nous avons testé les versions v5, v10 et v12.

- **Codes à tester** : dans le répertoire *algorithmes* il y a trois codes fonctionnants prêts à tester. Le fichier *limmswarm.hex* est le code plus complexe.

- **Simulateur kilombo** : l'installation se déroule comme prévu sur Ubuntu 20.04, mais apparemment elle échoue sur les versions plus récentes.
