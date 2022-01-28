# planej_de_movimento_trabalho_02

Este repositório contém pacotes do ROS que implementam os algoritmos de "A*", "GVD", "Trapezoidal Decomposition" e "PRM"..

Para utilizá-los é preciso:
1. instalar/copiar o pacote (catkin_ws/src/<pacote>) para seu workspace do ROS.
2. dar source no setup file do seu catkin_ws (lembre sempre de dar este source antes de executar quaisquer um dos pontos seguintes)
3. abrir um terminal e rodar "roscore"
4. inicializar o stage com o .world do algoritmo desejado (catkin_ws/src/<pacote>/worlds/<.world file>):
  rosrun stage_ros stageros -d ./catkin_ws/src/<pacote>/worlds/<.world file>
5. rodar o script do algoritmo desejado
  python3 catkin_ws/src/<pacote>/scripts/<algorithm .py script>
  
Os resultados serão salvos em um .csv file.

OBS: este projeto foi desenvolvido utilizando ROS Noetic, mas não acredito que haja incompatibilidade com outras versões do ROS. Python3 foi utilizado para o desenvolvimento, pode ser que hajam incompatibilidades caso use Python.

A implementação do "A*" realizada pelo Eudes, pode ser encontrada no arquivo "trab_questao1.zip". Favor descompactar o .zip e seguir suas orientações.
