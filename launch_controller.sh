
docker run -it --network=host  basmango/ibvs-mavros-controller-node:latest rosrun mavros mavsys rate --position 80 && python3 controller.py
 
