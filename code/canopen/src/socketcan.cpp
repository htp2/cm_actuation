
#include <socketcan.hpp>
#include <ros/console.h>

SocketCAN::SocketCAN( const std::string& devicename, 
		      SocketCAN::Rate rate,
		      SocketCAN::Loopback loopback ) : 
  devicename( devicename ),
  rate( rate ),
  loopback( loopback ),
  filterscnt( 0 ){
  
  // Check if the device name is empty
  if( devicename.empty() )
    { ROS_WARN_STREAM( "No device name." << std::endl ); }

}

SocketCAN::~SocketCAN(){}

SocketCAN::Errno SocketCAN::Open(){


  int errno;
  struct ifreq ifr;

  // create a CAN socket
  canfd = socket( PF_CAN, SOCK_RAW, CAN_RAW );
  if( canfd < 0 ){
    ROS_ERROR_STREAM( "Couldn't create a CAN socket." << std::endl );
    return SocketCAN::EFAILURE;
  }

  if( loopback == SocketCAN::LOOPBACK_ON ){
    errno = setsockopt( canfd, 
			SOL_CAN_RAW, 
			CAN_RAW_LOOPBACK,
			&loopback, 
			sizeof(loopback) );
    if( errno != 0) {
      ROS_ERROR_STREAM( "Couldn't set loopback mode for " << devicename
			<< ". Error code was: " << errno
			<< std::endl );
      return SocketCAN::EFAILURE;
    }
  }

  // Get CAN interface index by name
  strncpy(ifr.ifr_name, devicename.data(), IFNAMSIZ);
  errno = ioctl( canfd, SIOCGIFINDEX, &ifr );
  if( errno != 0 ){
    ROS_ERROR_STREAM( "Couldn't get the CAN interface index by name."
		      << "Error code was: " << errno
		      << std::endl );
    return SocketCAN::EFAILURE;
  }

  //! The socket address for the CAN address family
  struct sockaddr_can addr;

  // Bind the socket to the local address
  memset(&addr, 0, sizeof(addr));     // clear the address
  addr.can_ifindex = ifr.ifr_ifindex; // ifr_ifindex was set from SIOCGIFINDEX
  addr.can_family = AF_CAN;           // Address Family CAN

  errno = bind( canfd, (struct sockaddr*)&addr, sizeof(struct sockaddr_can) );

  if( errno != 0 ){
    ROS_ERROR_STREAM( "Couldn't bind the socket. Error code was: " 
		      << errno << std::endl );
    return EFAILURE;
  }

  return ESUCCESS;
}

SocketCAN::Errno SocketCAN::Close(){

  // close the socket
  if( close( canfd ) ){
    ROS_ERROR_STREAM( "Couldn't close the socket." << std::endl );
    return EFAILURE;
  }

  return ESUCCESS;
}

// Send a can frame
// Note that block is useless for Socket CAN
SocketCAN::Errno SocketCAN::Send( const CANFrame& canframe, SocketCAN::Flags ){
  // copy the data in to a Socket CAN frame
  // can_frame_t is defined in xenomai/include/rtdm/rtcan.h
  can_frame frame;
  frame.can_id = (canid_t)canframe.GetID();
  frame.can_dlc = (__u8)canframe.GetLength();  

  const __u8* data = (const __u8*)canframe.GetData();
  for(size_t i=0; i<8; i++)
    { frame.data[i] = data[i]; }

  // send the frame
  int error = send( canfd, (void*)&frame, sizeof(can_frame), 0 );
  if( error < 0 ){
    ROS_ERROR_STREAM( "Failed to send CAN frame " << error << std::endl );
    perror( "Failure " );
    return EFAILURE;
  }

  return ESUCCESS;
}

// Send a can frame
// Note that block is useless for Socket CAN
SocketCAN::Errno SocketCAN::SendRTR( const CANFrame& canframe, SocketCAN::Flags ){
  // copy the data in to a Socket CAN frame
  // can_frame_t is defined in xenomai/include/rtdm/rtcan.h
  can_frame frame;
  frame.can_id = (canid_t)canframe.GetID();
  // set the RTR bit
  frame.can_id |= CAN_RTR_FLAG;

  frame.can_dlc = (__u8)canframe.GetLength();  

  const __u8* data = (const __u8*)canframe.GetData();
  for(size_t i=0; i<8; i++)
    { frame.data[i] = data[i]; }

  // send the frame
  int error = send( canfd, (void*)&frame, sizeof(can_frame), 0 );
  if( error < 0 ){
    ROS_ERROR_STREAM( "Failed to send CAN frame " << error << std::endl );
    perror( "Failure " );
    return EFAILURE;
  }

  return ESUCCESS;
}

// Receive a CAN frame
SocketCAN::Errno SocketCAN::Recv( CANFrame& canframe, SocketCAN::Flags){
  struct can_frame frame;            // the RT Socket CAN frame
  memset(&frame, 0, sizeof(frame));  // clear the frame
  
  int error = recv( canfd, (void*)&frame, sizeof(can_frame), 0 );
  if( error < 0 ){
    perror( "error: " );
    ROS_ERROR_STREAM( "Failed to receive the frame. Error: " << error 
		      << std::endl );
    return EFAILURE;
  }

  // create a SocketCANFrame
  canframe = CANFrame( frame.can_id, frame.data, frame.can_dlc );

  return ESUCCESS;
}

SocketCAN::Errno SocketCAN::AddFilter( const CANFilter& filter ){

  if( filterscnt < MAX_NUM_FILTERS ){

    filters[filterscnt].can_mask = filter.mask;
    filters[filterscnt].can_id   = filter.id;
    filterscnt++;

    // Set the filter to the socket
    if( setsockopt( canfd, 
		    SOL_CAN_RAW, 
		    CAN_RAW_FILTER, 
		    filters, 
		    filterscnt*sizeof(struct can_filter) ) ){
      ROS_ERROR_STREAM( "Couldn't set the socket filters." << std::endl );
      return SocketCAN::EFAILURE;
    }

    return SocketCAN::ESUCCESS;
  }

  else{
    ROS_ERROR_STREAM( "Reached maximum number of filters." << std::endl );
    return SocketCAN::EFAILURE;
  }

  return SocketCAN::EFAILURE;

}



