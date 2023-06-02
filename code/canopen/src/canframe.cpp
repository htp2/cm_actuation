#include <canframe.hpp>
#include <ros/console.h>

// Default initialization of a CAN frame
CANFrame::CANFrame(){ 
  // Clear up everything
  this->id = 0;                                  // default ID 
  this->nbytes=0;                                // no data
  for(CANFrame::DataLength i=0; i<8; i++) // clear the data
    { this->data[i] = 0x00; }
}

CANFrame::CANFrame( CANFrame::ID id, 
		    CANFrame::DataField data,
		    CANFrame::DataLength nbytes ){

  // Clear up everything before starting
  this->id = 0;                                  // default ID 
  this->nbytes = 0;                              // no data
  for(CANFrame::DataLength i=0; i<8; i++) // clear the data
    { this->data[i] = 0x00; }

  // A can ID has 11 bits. Ensure that only 11 bits are used
  if( (~0x07FF) & id )
    { ROS_WARN_STREAM( "Illegal CAN id: " << id << std::endl ); }

  else{
    // Now check that no more than 8 bytes are given
    if( 8 < nbytes )
      { ROS_WARN_STREAM( "Illegal message length: " << nbytes << std::endl ); }

    else{
      this->id = (0x07FF & id);                          // Copy the CAN ID
      this->nbytes = nbytes;                             // Copy the data length
      for(CANFrame::DataLength i=0; i<nbytes; i++)// Copy the data
	{ this->data[i] = data[i]; }
    }

  }

}

CANFrame::CANFrame( CANFrame::ID id, 
		    const std::vector<CANFrame::Data>& data ){

  // Clear up everything before starting
  this->id = 0;                              // default ID 
  this->nbytes = 0;                          // no data
  for(CANFrame::DataLength i=0; i<8; i++) // clear the data
    { this->data[i] = 0x00; }

  // A can ID has 11 bits. Ensure that only 11 bits are used
  if( (~0x07FF) & id )
    { ROS_WARN_STREAM( "Illegal CAN id: " << id << std::endl ); }

  else{
    // Now check that no more than 8 bytes are given
    if( 8 < data.size() )
      { ROS_WARN_STREAM( "Illegal message length: " << data.size() ); }

    else{
      this->id = (0x07FF & id);                         // Copy the CAN ID
      this->nbytes = data.size();                       // Copy the data length
      for(CANFrame::DataLength i=0; i<nbytes; i++)      // Copy the data
	{ this->data[i] = data[i]; }
    }
  }

}

