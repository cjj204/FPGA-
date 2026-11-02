if { [catch { source ./camera_to_dsi_display.tcl } err] } {
    puts ""
    puts "################ FLOW_ERROR ################"
    puts $err
    puts "############################################"
}
puts ""
puts "################ FLOW_DONE ################"
exit