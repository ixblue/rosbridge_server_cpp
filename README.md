# `p_rosbridge_server_cpp`

Implémentation C++ d'un serveur Websocket du [protocole Rosbridge](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md).

Il utilise Qt pour l'implémentation Websocket et donc dépend de `libqt5websockets5-dev`.

Les "compressions" **JSON**, **CBOR** et **CBOR-RAW** sont supportés mais pas la compression **PNG**.

Pas implémenté par rapport à la version Python :

- compression PNG
- paquets fragmentés
- authentification (`rosauth`)
- service server
- Autres transports que Websocket (TCP, UDP)
- Mode BSON only

## Dépendances

- `libqt5websockets5-dev`
- `roscpp`
- `librosqt`
- `ros_babel_fish`
- `rosbridge_msgs`
- `std_msgs`

## Paramètres optionels

- `port` (*int*) : Port de la websocket. La valeur 0 conduit à ce que l'OS donne un port (défaut = `9090`)
- `service_timeout` (*double*) : Durée du timeout pour les appels de services en secondes (défaut = `5.0`)

## Paramètres set par le nœud

Le nœud set lui-même le paramètre `actual_port`, utile dans le cas où `port` est mis à `0`.

## Topics publiés

Topics hérités du nœud Python :

- `client_count` (*std_msgs/Int32*) : Nombre de clients connectés, latché et mis à jour à chaque changement du nombre de clients.
- `connected_clients` (rosbridge_msgs/ConnectedClients**) : Information sur les clients actuellement connectés, latché et mis à jour à chaque changement du nombre de clients.
