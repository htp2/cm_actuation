
#include <canopen.hpp>
#include <ros/console.h>

//#include <fcntl.h>

CANopen::CANopen( const std::string& devicename,
		  SocketCAN::Rate rate,
		  SocketCAN::Loopback loopback ) :
  socketcan( devicename, rate, loopback ), 
  deviceopened( false ){}

CANopen::~CANopen(){}

CANopen::Errno CANopen::Open(){

  if( socketcan.Open() != SocketCAN::ESUCCESS ){
    ROS_ERROR_STREAM( "Failed to open CAN device." << std::endl );
    return CANopen::EFAILURE;
  }
  
  deviceopened = true;
  return CANopen::ESUCCESS;
}

CANopen::Errno CANopen::Close(){

  if( socketcan.Close() != SocketCAN::ESUCCESS ){
    ROS_ERROR_STREAM( "Failed to close CAN device." << std::endl );
    return CANopen::EFAILURE;
  }
  
  deviceopened = false;
  return CANopen::ESUCCESS;

}

CANopen::Errno CANopen::Read( CiA301::COBID& cobid, CiA301::Object& object ){
   
   
  CANFrame frame;
  if( socketcan.Recv( frame ) != SocketCAN::ESUCCESS ){
    ROS_ERROR_STREAM( "Failed to read CAN frame." << std::endl );
    return CANopen::EFAILURE;
  }

  Unpack( frame, cobid, object );
   
  return CANopen::ESUCCESS;
}

CANopen::Errno CANopen::Write( CiA301::COBID cobid, 
			       const CiA301::Object& object ){
   
   if( socketcan.Send( Pack( cobid, object ) ) != SocketCAN::ESUCCESS ){
     ROS_ERROR_STREAM( "Failed to write CAN frame." << std::endl );
     return CANopen::EFAILURE;
   }

   return CANopen::ESUCCESS;

}

// Pack
// 
CANFrame CANopen::Pack( CiA301::COBID cobid, const CiA301::Object& object )
{ return CANFrame( cobid, object.data.data ); }

// Unpack
// 
void CANopen::Unpack( const CANFrame& frame,
		      CiA301::COBID& cobid,
		      CiA301::Object& object ){

  const CANFrame::Data* src = frame.GetData();
  CiA301::Object::DataField dest( size_t(frame.GetLength()) );

  for( unsigned char i=0; i<frame.GetLength(); i++ )
    { dest[i] = src[i]; }
   
  cobid = frame.GetID();
  object = CiA301::Object( dest );

}

