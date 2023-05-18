
#ifndef _MAXON_HPP_
#define _MAXON_HPP_

namespace Maxon
{

  // this object is not CANopen standard
  struct HomingCurrentThreshold : public CiA301::SDO {
    HomingCurrentThreshold( unsigned short current ) : 
      CiA301::SDO( CiA301::SDO::INITIATE_WRITE, 0x2080, 0, current ){}
  };
  
  // COB ids for the PDO used at CNMC
  namespace COBID
  {
    
    const CiA301::COBID TPDO1       = 0x180;
    const CiA301::COBID TPDO1_LOW   = 0x181;
    const CiA301::COBID TPDO1_HIGH  = 0x186;
      
    const CiA301::COBID TPDO2       = 0x190;
    const CiA301::COBID TPDO2_LOW   = 0x191;
    const CiA301::COBID TPDO2_HIGH  = 0x196;
    
    const CiA301::COBID TPDO3       = 0x200;
    const CiA301::COBID TPDO3_LOW   = 0x201;
    const CiA301::COBID TPDO3_HIGH  = 0x206;
    
    const CiA301::COBID TPDO4       = 0x210;
    const CiA301::COBID TPDO4_LOW   = 0x211;
    const CiA301::COBID TPDO4_HIGH  = 0x216;
    
    const CiA301::COBID TPDO5       = 0x220;
    const CiA301::COBID TPDO5_LOW   = 0x221;
    const CiA301::COBID TPDO5_HIGH  = 0x226;
    
    const CiA301::COBID RPDO2       = 0x400;
    const CiA301::COBID RPDO2_LOW   = 0x401;
    const CiA301::COBID RPDO2_HIGH  = 0x406;
    
    const CiA301::COBID RPDO3       = 0x410;
    const CiA301::COBID RPDO3_LOW   = 0x411;
    const CiA301::COBID RPDO3_HIGH  = 0x416;
    
    const CiA301::COBID RPDO4       = 0x420;
    const CiA301::COBID RPDO4_LOW   = 0x421;
    const CiA301::COBID RPDO4_HIGH  = 0x426;
    
    const CiA301::COBID RPDO5       = 0x430;
    const CiA301::COBID RPDO5_LOW   = 0x431;
    const CiA301::COBID RPDO5_HIGH  = 0x436;
    
  }

}

#endif
