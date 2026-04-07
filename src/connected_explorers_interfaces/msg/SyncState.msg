uint8 PHASE_WAITING=0
uint8 PHASE_CALCULATING=1
uint8 PHASE_CONSENSUS=2
uint8 PHASE_READY=3
uint8 PHASE_MOVING=4

string robot_id
uint8 current_phase
uint32 move_epoch
uint32 micro_step
uint32 consensus_step
float64[] d_k_row
float64 current_max_diff
float64 fiedler_vector_comp