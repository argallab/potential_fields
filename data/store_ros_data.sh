#!/bin/bash

# This script stores ROS dataflow diagram in a markdown file.
# With the headers: NODES, TOPICS, PUB/SUB, SERVICES, MSGS, SRVS

OUTPUT_FILE="ros_structure.txt"

list_nodes() {
  ros2 node list | while read -r node; do
    # Skip nodes matching transform_listener_impl_*
    if [[ "$node" == *"transform_listener_impl_"* ]]; then
      continue
    fi
    ns=$(ros2 node info "$node" | grep "Namespace:" | awk '{print $2}')
    type=$(ros2 node info "$node" | grep "Node Name:" | awk '{print $3}')
    echo "node: $node  (ns: $ns, type: $type)"
  done
}

list_topics() {
    ros2 topic list | while read -r topic; do
        type=$(ros2 topic info "$topic" | grep "Type:" | awk '{print $2}')
        echo "topic: $topic (msg: $type)"
    done
}

# Desired output:
# node1 -> /topic1 (publishes)
# node2 <- /topic1 (subscribes)
# node3 -> /topic2 (publishes)
# ...
list_pub_sub() {
  skip_topics=(
    "parameter_events"
    "rosout"
    "rosout_agg"
    "clock"
  )

  ros2 topic list | while read -r topic; do
    base="${topic##*/}"   # get last segment (basename)
    skip=false
    for s in "${skip_topics[@]}"; do
      if [[ "$base" == "$s" ]]; then
        skip=true
        break
      fi
    done
    if $skip; then
      continue
    fi

    ros2 topic info -v "$topic" 2>/dev/null | awk -v t="$topic" '
      /^Endpoint type:/ { et=$3 }
      /^Node name:/     { nn=$3 }
      /^Node namespace:/{ ns=$3
        # Skip nodes matching transform_listener_impl_*
        if (nn ~ /^transform_listener_impl_/) next;
        if (et=="PUBLISHER")
          printf "%s%s -> %s (publishes)\n", ns, nn, t;
        else if (et=="SUBSCRIPTION")
          printf "%s%s <- %s (subscribes)\n", ns, nn, t;
        et=""; nn=""; ns="";
      }'
  done
}


list_services() {
  # List of default ROS services to skip
skip_services=(
  "/describe_parameters"
  "/get_parameter_types"
  "/get_parameters"
  "/get_type_description"
  "/list_parameters"
  "/set_parameters"
  "/set_parameters_atomically"
)

ros2 service list | while read -r service; do
  skip=false
  for s in "${skip_services[@]}"; do
    if [[ "$service" == *"$s" ]]; then  # suffix match
      skip=true
      break
    fi
  done
  if ! $skip; then
    type=$(ros2 service type "$service" 2>/dev/null)
    echo "service: $service (srv: $type)"
  fi
done
}


# If the file exists, clear it
if [ -f "$OUTPUT_FILE" ]; then
    > "$OUTPUT_FILE"
fi

echo "# ROS Dataflow Diagram" > $OUTPUT_FILE
echo "" >> $OUTPUT_FILE
echo "# NODES" >> $OUTPUT_FILE
list_nodes >> $OUTPUT_FILE
echo "" >> $OUTPUT_FILE
echo "# TOPICS" >> $OUTPUT_FILE
list_topics >> $OUTPUT_FILE
echo "" >> $OUTPUT_FILE
echo "# PUB/SUB" >> $OUTPUT_FILE
list_pub_sub >> $OUTPUT_FILE
echo "" >> $OUTPUT_FILE
echo "# SERVICES" >> $OUTPUT_FILE
list_services >> $OUTPUT_FILE
echo "" >> $OUTPUT_FILE

echo "ROS dataflow diagram saved to $OUTPUT_FILE"
