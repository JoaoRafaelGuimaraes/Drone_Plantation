# MRS Drone Planter (Fork Atualizado)

Este repositório demonstra um exemplo de simulação de plantio automatizado com um drone no Gazebo + ROS. Ao contrário da versão original, **a câmera não está sendo utilizada** por enquanto. Em vez disso, o sistema foca em dois nós principais em Python:

1. **PlanterNode** (drone_planter.py)  
   - Gera uma trajetória fixa de waypoints (mudas) para o drone voar a uma altitude de cruzeiro (2,5 m).  
   - Ao chegar em cada ponto, reduz para a altitude de plantio (1,0 m), “planta” no Gazebo (spawn de modelo SDF) e emite um alerta (`/muda_alert`).  
   - Após plantar, retorna à altitude de cruzeiro e segue para o próximo ponto.

2. **KMLGenerator** (kml_generator.py)  
   - Escuta o tópico `/muda_alert` para saber quando uma muda foi “plantada”.  
   - Consulta as coordenadas GPS atuais do drone (tópico `/uav1/mavros/global_position/global`) e registra em memória uma lista de tuplas `(nome_muda, latitude, longitude)`.  
   - Ao encerrar (shutdown do nó), grava todos os pontos registrados em um arquivo KML (pasta `/root/KMLs/`), nomeado como `kml_YYYYMMDD_HHMMSS.kml`.

---

## 1. Descrição Geral

O fluxo de simulação funciona assim:

1. O nó **PlanterNode** é inicializado e, após aguardar 1 s, envia ao controlador de voo (via serviço `/uav1/trajectory_generation/path`) uma lista fixa de pontos de plantio (cada ponto contém `x`, `y` e um nome de muda).  
2. O drone decola automaticamente e segue a rota em voo de cruzeiro (2,5 m de altura).  
3. Quando o odometria do drone (tópico `/uav1/estimation_manager/odom_main`) indica que ele chegou num ponto (distância < 1 m), o PlanterNode:  
   a. Chama o serviço `/uav1/control_manager/goto` para manter o drone exatamente em `(x, y, 2,5)`.  
   b. Reduz a altitude para 1,0 m (planting) via outro chamado a `/uav1/control_manager/goto`.  
   c. Faz spawn de um modelo SDF de planta no Gazebo (arquivo `models/big_plant/model.sdf`).  
   d. Publica o nome da muda em `/muda_alert`.  
   e. Retorna à altura de cruzeiro (2,5 m) e prossegue para o próximo ponto.  
4. O nó **KMLGenerator** recebe cada nome de muda via `/muda_alert`, lê o GPS atual (via tópico `/uav1/mavros/global_position/global`) e guarda `(nome, lat, lon)` numa

## 2. Execução

Para executar o programa, execute o arquivo presente em '/python/blob_detector/tmux/start.sh'. Após isso, execute o arquivo launch com o comando 'roslaunch example_blob_detector planta_muda.launch'
