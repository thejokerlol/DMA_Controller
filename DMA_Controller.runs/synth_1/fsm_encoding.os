
 add_fsm_encoding \
       {DMA_Controller.axi_read_state} \
       { }  \
       {{000 00} {001 11} {010 10} {011 01} }

 add_fsm_encoding \
       {DMA_Controller.next_thread_to_execute} \
       { }  \
       {{0000 0000} {0001 0001} {0010 0010} {0011 0011} {0100 0100} {0101 0101} {0110 0110} {0111 0111} {1000 1000} }
