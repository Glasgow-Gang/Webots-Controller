# Webots-Controller

Using as extern controller in Webots.

## Install SDL

```bash
sudo apt install libsdl2-ttf-dev libsdl2-mixer-dev libsdl2-image-dev libsdl2-ttf-dev libsdl2-dev
```

## Build

```bash
mkdir build
cd build
cmake ..
make
```

## Run

```bash
export WEBOTS_HOME=/usr/local/webots
bash -c "nohup /usr/local/webots/webots-controller --robot-name=NAO_2_0 ./xrobot attacker 1 &" && bash -c "nohup /usr/local/webots/webots-controller --robot-name=NAO_1_0 ./xrobot defender1 1 &" && bash -c "nohup /usr/local/webots/webots-controller --robot-name=NAO_3_0 ./xrobot defender2 1 &" && bash -c "nohup /usr/local/webots/webots-controller --robot-name=NAO_4_0 ./xrobot goalkeeper 1 &" && bash -c "nohup /usr/local/webots/webots-controller --robot-name=NAO_2_1 ./xrobot attacker 2 &" && bash -c "nohup /usr/local/webots/webots-controller --robot-name=NAO_1_1 ./xrobot defender1 2 &" && bash -c "nohup /usr/local/webots/webots-controller --robot-name=NAO_3_1 ./xrobot defender2 2 &" && bash -c "nohup /usr/local/webots/webots-controller --robot-name=NAO_4_1 ./xrobot goalkeeper 2 &"
```
