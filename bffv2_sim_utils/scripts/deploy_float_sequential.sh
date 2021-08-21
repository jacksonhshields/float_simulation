#!/bin/bash

current_vel=0.5 # m/s
current_angle=-90 # degrees, CCW from east

deploy_set=$1

usage()
{
    echo "USAGE:"
    echo "./deploy_float_sequential.sh deploy_set.csv"
}

add_mission()
{
    float_name=$1
    duration=$2

    echo "Adding mission for ${float_name} for ${duration} seconds"

#     rosservice call /${float_name}/add_mission "mission_stamped:
#   header:
#       seq: 0
#       stamp:
#       secs: 0
#       nsecs: 0
#       frame_id: ''
#   mission:
#       mission_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}
#       float_name: ''
#       altitude: 5.0
#       bottom_duration: ${duration}
#       mission_duration: ${duration}
#       dive_abort_duration: 1000.0
#       abort_depth: 40.0
#       abort_time: {secs: 0, nsecs: 0}"

    rosservice call /float2/add_mission "mission_stamped:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  mission:
    mission_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}
    float_name: ''
    altitude: 5.0
    bottom_duration: ${duration}
    mission_duration: ${duration}
    dive_abort_duration: 1000.0
    abort_depth: 300.0
    abort_time: {secs: 0, nsecs: 0}"

}

deploy_float()
{
    float_name=$1
    rosservice call /${float_name}/notify_deploying "wait: 1.0"
}

recover_float()
{
    float_name=$1
    rosservice call /${float_name}/notify_recovered "surface_vessel_name: 'workboat1'"
}

go_to_surface()
{
    rosservice call /set_depth_target "depth_target: 0.0
timeout: 20.0"
}

float_mission()
{
    float_name=$1
    lat=$2
    lon=$3
    duration=$4
    xloc=$5
    yloc=$6


    wait_time=$(($duration + 60))

    if test -z $xloc; then
        # Use lat, lon if x,y not given
        roslaunch bffv2_description float_simulation.launch use_geodetic:=true latitude:=${lat} longitude:=${lon} float_name:=${float_name} &
        pid=$!
        # ^ use timeout instead???
    else
        roslaunch bffv2_description float_simulation.launch use_geodetic:=false x_loc:=${xloc} y_loc:=${yloc} float_name:=${float_name} &
        pid=$!

    fi
    
    sleep 10
    add_mission ${float_name} ${duration}

    sleep 5
    deploy_float ${float_name}


    sleep $wait_time

    go_to_surface

    sleep 20

    recover_float ${float_name}

    sleep 10

    echo "Killing current mission"
    kill $pid

    sleep 5
    echo "Deleting the model: ${float_name}"
    rosservice call gazebo/delete_model "{model_name: ${float_name}}"


}

if test -z $deploy_set; then
    usage
    exit 1
fi

# roslaunch uuv_control_utils set_timed_current_perturbation.launch starting_time:=0.0 end_time:=10000.0 current_vel:=${current_vel} horizontal_angle:=${current_angle} &

cat $deploy_set | while read line 
do
   # line format: deployment_name,lat,lon,dive_time,float_name,x,y
   lat=$(echo $line | cut -d, -f2)
   lon=$(echo $line | cut -d, -f3)
   dive_time=$(echo $line | cut -d, -f4)
   float_name=$(echo $line | cut -d, -f5)
   deployment_name=$(echo $line | cut -d, -f1)
   xloc=$(echo $line | cut -d, -f6 )
   yloc=$(echo $line | cut -d, -f7 )
   echo "Running mission $deployment_name for ${dive_time} seconds"
   float_mission ${float_name} ${lat} ${lon} ${dive_time} ${xloc} ${yloc}
done

