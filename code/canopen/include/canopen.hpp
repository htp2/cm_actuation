

#ifndef _CANOPEN_HPP_
#define _CANOPEN_HPP_

#include <socketcan.hpp>
#include <cia301.hpp>

class CANopen{

 public:
   
   enum Errno
   {
     ESUCCESS = 0x00,
     EFAILURE = 0x01
   };

 private:

  SocketCAN socketcan;
  bool deviceopened;
   
  CANFrame Pack( CiA301::COBID cobid, const CiA301::Object& object );
  void Unpack( const CANFrame& frame, CiA301::COBID& cobid, CiA301::Object& object );
   
 public:
   
  CANopen( const std::string& devicename,
	   SocketCAN::Rate rate,
	   SocketCAN::Loopback loopback );
  ~CANopen();
   
  CANopen::Errno Open();
  CANopen::Errno Close();

   /*! \brief Read CAN message from CAN driver */
  CANopen::Errno Read( CiA301::COBID& cobid, CiA301::Object& object );

   /*! \brief Write CAN frame. */
  CANopen::Errno Write( CiA301::COBID cobid, const CiA301::Object& object );
  CANopen::Errno WriteRTR( CiA301::COBID cobid);

  bool IsOpen(void) const { return deviceopened; }
  
};

#endif
