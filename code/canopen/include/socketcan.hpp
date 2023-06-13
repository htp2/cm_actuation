#ifndef _SOCKETCAN_HPP_
#define _SOCKETCAN_HPP_

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <linux/if.h>

#include <canframe.hpp> 
#include <canfilter.hpp> 

//! A Real Time Socket CAN device
/**
   Most harware in Linux use the /dev interface. Typically, your device (usb,
   serial port, ...) will be represented by a file in /dev. CAN hardware is no
   exception except for the socket CAN module. SocketCAN implements a CAN stack
   and extend the BSD sockets just like you have a ethernet stack and sockets. 
   RT SocketCAN is the "real time" version for the Xenomai framework.
   To use SocketCAN, you must have a Xenomai patched Linux kernel with 
   The RT CAN module enabled
*/

class SocketCAN{

public:
  
  enum Rate { RATE_150 =150000, 
	      RATE_300 =300000, 
	      RATE_1000=1000000 };

  enum Errno { ESUCCESS, EFAILURE };

  enum Flags{ MSG_NOFLAG   = 0x00,
	      MSG_CONFIRM  = 0x01,   // ask for a confirmation
	      MSG_DONTWAIT = 0x02 }; // enables non-blocking operation
  
  enum Loopback{ LOOPBACK_ON, LOOPBACK_OFF };

private:
  
  //! The name of the CAN device (rtcan0, rtcan1, ...)
  std::string devicename;

  SocketCAN::Rate rate;
  SocketCAN::Loopback loopback;  
  
  //! The file descriptor of the socket
  int canfd;
  
  //! CAN filters
  /**
     The default maximum number of CAN filter in Xenomai is 16. Howerver, this
     limit can be increased when configuring the kernel.
   */
  static const size_t MAX_NUM_FILTERS = 32;
  struct can_filter filters[MAX_NUM_FILTERS];
  size_t filterscnt;

public:

  //! Constructor
  /**
     Initialize the device name and the rate of the CAN device
     \param devicename The name of the device (rtcan0, rtcan1, ...)
     \param rate The CAN rate (RATE_150, RATE_300, RATE_1000)
  */
  SocketCAN( const std::string& devicename, 
	     SocketCAN::Rate rate,
	     SocketCAN::Loopback loopback = SocketCAN::LOOPBACK_OFF );
  
  //! Destructor
  ~SocketCAN();

  //! Open and configure the CAN socket
  /**
     Creates and configure a RT CAN socket.
  */
  SocketCAN::Errno Open();

  //! Close the CAN socket
  SocketCAN::Errno Close();

  //! Send a CAN frame on the bus
  /**
     Call this method to send a CAN frame on the bus.
     \param frame[in] The CAN frame to send on the bus
     \param flags Block the device until the operation is completed. This
                  parameter is irrelevant for SocketCAN.
  */
  SocketCAN::Errno Send( const CANFrame& frame, 
			 SocketCAN::Flags flags = SocketCAN::MSG_NOFLAG );

  SocketCAN::Errno SendRTR( const CANFrame& frame, 
			 SocketCAN::Flags flags = SocketCAN::MSG_NOFLAG );

  //! Receive a CAN frame
  /**
     Call this method to receive a CAN frame.
     \param frame[out] The CAN frame received from the bus
     \param flags Block the device until a CAN frame is received. This
                  parameter is irrelevant for SocketCAN.
  */
  SocketCAN::Errno Recv( CANFrame& frame, 
			 SocketCAN::Flags flags = SocketCAN::MSG_NOFLAG );
  
  //! Add a filter to the SocketCAN device
  SocketCAN::Errno AddFilter( const CANFilter& filter );

};

#endif
