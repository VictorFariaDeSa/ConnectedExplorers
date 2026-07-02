# ConnMessage.msg
uint8 sender_id             # Filled by Outbox
uint8 receiver_id               # Who is sending this
float64 conn_weight
uint16 node_count           # 'N' - The total number of nodes in the matrix

geometry_msgs/Pose  pose      # The robot's current position and orientation
float64[] adjacency_data    # The flattened 1D array of length N(N-1)/2