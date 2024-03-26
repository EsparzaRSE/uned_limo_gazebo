# Problema con la Odometría

Revisar la odometría, se vuelve loca cuando en cuanto el robot gira (ya está corregido, creo, faltan pruebas)

# Descartado usar directamente desde xacro

Desde el xacro directamente muchas propiedades no se convierten cuando lo hace sdf al spawnear el robot en ignition, como la fricción. Eso hace que el robot no se mueva bien. Para arreglarlo tienes que poner todas estas propiedades a mano en un sdf o no funcionan.

# Navegación

De momento en modo diferencial funciona y si le añades nuevos objetos de la nada actualiza el mapa y la ruta.
En modo ackerman tengo que averiguar como funciona porque intenta girar como un diferencial y no llega al objetivo.
Y el omnidireccional también tengo que probarlo a ver que tal.
