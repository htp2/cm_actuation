
#ifndef _CANFILTER_HPP_
#define _CANFILTER_HPP_

#include <canframe.hpp>

// CAN Filter
/**
   CAN filters are used by the kernel to filter frames that are not aimed for
   a specific node id.
*/

class CANFilter {
  
public:
  
  CANFrame::Mask mask;
  CANFrame::ID id;
  
  CANFilter( CANFrame::Mask mask, CANFrame::ID id ) : mask( mask ), id( id ) {}
  
};

#endif
