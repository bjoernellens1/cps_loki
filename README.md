# cps_loki main repo

## Now working via Docker!
Follow the instructions: To use this you need the latest Docker installed and for the multiplatform build the binfmt dependencies for your system. Otherwise just use the pre-built Docker containers. I will try to make the images available public.

TODO: rename packages!
First let's check if you are able to pull the pre-built images:
```
    docker pull ghcr.io/bjoernellens1/bot_mini_ws/bot:overlay
    docker pull ghcr.io/bjoernellens1/bot_mini_ws/bot:guis
```

Run:
For instance on your PC:
```
    docker compose run guis
```

This will launch the "guis" container where you will find a full ros2 humble installation and all the dependencies this robot needs.

On the robot:
```
    docker compose up -d controller teleop
```

So as you can see, you will be working from the same git repository as well on the robot and your dev PC. 
For the fact all can run inside Docker makes the project super portable so everyone should be able to tinker around with it. Nice bonus perks: You won't interfere with your computer's local environment. With Docker you are even able to run multiple versions of ROS simultanously, even on unsupported Linux operating systems like Debian or NixOS. Also on MacOS. Windows might be possible but not tested.


### Attention: do not launch docker compose without arguments else you will start all services at once and you won't need them.

For building your own images:
I would advise to fork the repository and start working in your own one.
```
    docker buildx bake overlay --load
```
This will allow you to modify the images to your needs. Enjoy!

## Useful commands:
```
rosdep install --from-paths src --ignore-src -r -y
```
