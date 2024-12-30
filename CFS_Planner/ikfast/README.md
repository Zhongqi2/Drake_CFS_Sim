rosrun collada_urdf urdf_to_collada ../model/gp7_motoman.urdf ../model/gp7_motoman.dae
openrave-robot.py gen3.dae --info links
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=../model/gp7_motoman.dae --iktype=transform6d --baselink=1 --eelink=8 --savefile=ikfast61.cpp --maxcasedepth 5

apt install liblapack-dev

// for each ikfast_*
#julia> using CxxWrap; CxxWrap.prefix_path()
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DJulia_PREFIX=/opt/julia-1.10.3/ -DCMAKE_PREFIX_PATH=/root/.julia/artifacts/0c7b615ac941d356502c54c3f343cc97f247f3c1 ../
cmake --build . --config Release