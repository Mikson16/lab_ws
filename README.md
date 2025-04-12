# Lab de Ros2 para Robotica Movil del grupo de Julian Alberto


## Forma de instalacion del workspace
En bash 
cd ~
mkdir -p lab_ws/src
cd lab_ws
colcon build
source install/setup.bash (desde la carpeta lab_ws)

Luego desde la carpeta de lab_ws ejecutar : 
- git init


Se debera crear un token de acceso personal para utilizar ssh (preferible 90 dias como minimo)
- https://github.com/settings/personal-access-tokens
- generate new token
- Token name : Token para lab_ws
- Selecionar todos los repositorios (Para facilitar la necesidad de token futuros en sus propios repositorios)
- Esto es por en caso de que se los pida, copien el token y guardenlo por si lo necesitan, ya que luego desaparecera

Para configurar una clave ssh:
- ssh-keygen -t ed25519 -C "tu_email@example.com"
- Seleccionar yes siempre que lo pida y si no solo presionar enterhasta que se genere la clave
- Para agregar la clave ssh a github: cat ~/.ssh/id_ed25519.pub esto les dara la info, copienla.
- Se dirigen a esta pagina y crean su clave ssh https://github.com/settings/keys, en ella pegan la informacion que copiaron.




Ahora para agregar el repositorio a sus computadores:
- cd $HOME
- git clone https://github.com/Mikson16/lab_ws.git
- cd lab_ws
    source /opt/ros/humble/setup.bash
    colcon build
- source install/setup.bash
- ros2 pkg list (para revisar si les aparece el pkg lab_1_pkg)
- Recordar siempre dar permiso a los archivos con chmod u+x nombre.py, en la carpeta de scripts 
