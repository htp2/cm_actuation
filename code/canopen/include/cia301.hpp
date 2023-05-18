
#ifndef _CIA301_HPP_
#define _CIA301_HPP_

#include <vector>

namespace CiA301{
  
  typedef unsigned short COBID;
  
  namespace Node
  {
    typedef unsigned char ID;
    static const CiA301::Node::ID NODE_0 = 0x000;
    static const CiA301::Node::ID NODE_1 = 0x001;
    static const CiA301::Node::ID NODE_2 = 0x002;
    static const CiA301::Node::ID NODE_3 = 0x003;
    static const CiA301::Node::ID NODE_4 = 0x004;
    static const CiA301::Node::ID NODE_5 = 0x005;
  };
  
  
  // An object has one index and one sub index
  struct Object{
    
  public:
    
    typedef unsigned char  Data;
    struct DataField{
      
      std::vector< CiA301::Object::Data > data;

      const CiA301::Object::Data& operator[]( int i ) const 
      { return data.at( i ); }

            CiA301::Object::Data& operator[]( int i )
      { return data.at( i ); }

      size_t size() const { return data.size(); }

      DataField(){}

      DataField( size_t N ) : data( N ){}

      DataField( CiA301::Object::Data x1 ){
	data.push_back( x1 );
      }

      DataField( CiA301::Object::Data x1,
		 CiA301::Object::Data x2 ){
	data.push_back( x1 );
	data.push_back( x2 );
      }

      DataField( CiA301::Object::Data x1,
		 CiA301::Object::Data x2,
		 CiA301::Object::Data x3 ){
	data.push_back( x1 );
	data.push_back( x2 );
	data.push_back( x3 );
      }

      DataField( CiA301::Object::Data x1,
		 CiA301::Object::Data x2,
		 CiA301::Object::Data x3,
		 CiA301::Object::Data x4 ){
	data.push_back( x1 );
	data.push_back( x2 );
	data.push_back( x3 );
	data.push_back( x4 );
      }

      DataField( CiA301::Object::Data x1,
		 CiA301::Object::Data x2,
		 CiA301::Object::Data x3,
		 CiA301::Object::Data x4,
		 CiA301::Object::Data x5 ){
	data.push_back( x1 );
	data.push_back( x2 );
	data.push_back( x3 );
	data.push_back( x4 );
	data.push_back( x5 );
      }

      DataField( CiA301::Object::Data x1,
		 CiA301::Object::Data x2,
		 CiA301::Object::Data x3,
		 CiA301::Object::Data x4,
		 CiA301::Object::Data x5,
		 CiA301::Object::Data x6 ){
	data.push_back( x1 );
	data.push_back( x2 );
	data.push_back( x3 );
	data.push_back( x4 );
	data.push_back( x5 );
	data.push_back( x6 );
      }

      DataField( CiA301::Object::Data x1,
		 CiA301::Object::Data x2,
		 CiA301::Object::Data x3,
		 CiA301::Object::Data x4,
		 CiA301::Object::Data x5,
		 CiA301::Object::Data x6,
		 CiA301::Object::Data x7 ){
	data.push_back( x1 );
	data.push_back( x2 );
	data.push_back( x3 );
	data.push_back( x4 );
	data.push_back( x5 );
	data.push_back( x6 );
	data.push_back( x7 );
      }

      DataField( CiA301::Object::Data x1,
		 CiA301::Object::Data x2,
		 CiA301::Object::Data x3,
		 CiA301::Object::Data x4,
		 CiA301::Object::Data x5,
		 CiA301::Object::Data x6,
		 CiA301::Object::Data x7,
		 CiA301::Object::Data x8 ){
	data.push_back( x1 );
	data.push_back( x2 );
	data.push_back( x3 );
	data.push_back( x4 );
	data.push_back( x5 );
	data.push_back( x6 );
	data.push_back( x7 );
	data.push_back( x8 );
      }
      
    };

    CiA301::Object::DataField data;
    
    Object(){} 
    Object( const CiA301::Object::DataField&  data ) : data( data ) {}
    
    friend std::ostream& operator<<( std::ostream& os, 
				     const CiA301::Object& object ){
      for( size_t i=0; i<object.data.size(); i++ ){
	os << "0x" << std::hex << std::setfill('0') << std::setw(2) 
	   << (int)(object.data[i]) << " ";
      }
      os << std::dec;
      return os;
    }

  };
  
  struct SYNC : public CiA301::Object{ SYNC() : Object(){} };
  
  struct NMT : public CiA301::Object
  {
    static const CiA301::COBID COBID = 0x000;
    
    enum State {
      OPERATIONAL         = 0x01,
      STOPPED             = 0x02,
      PRE_OPERATIONAL     = 0x80,
      RESET_NODE          = 0x81,
      RESET_COMMUNICATION = 0x82
    };
    
    NMT(CiA301::NMT::State state, CiA301::Node::ID nodeid) :
      Object( DataField( state, nodeid ) ) {}
  };   
  
  struct NMTOperational : public CiA301::NMT {
    NMTOperational( CiA301::Node::ID nodeid ):
      NMT( CiA301::NMT::OPERATIONAL, nodeid ){}
  };
  
  struct NMTStop : public CiA301::NMT {
    NMTStop( CiA301::Node::ID nodeid ): 
      NMT( CiA301::NMT::STOPPED, nodeid ){}
  };
  
  struct NMTPreOperational : public CiA301::NMT {
    NMTPreOperational( CiA301::Node::ID nodeid ): 
      NMT( CiA301::NMT::PRE_OPERATIONAL, nodeid ){}
  };
  
  struct NMTResetNode : public CiA301::NMT {
    NMTResetNode( CiA301::Node::ID nodeid ): 
      NMT( CiA301::NMT::RESET_NODE, nodeid ){}
  };
  
  struct NMTResetCommunication : public CiA301::NMT {
    NMTResetCommunication( CiA301::Node::ID nodeid ):
      NMT( CiA301::NMT::RESET_COMMUNICATION, nodeid ){}
  };
  
  
  namespace Emergency
  {
    
    const CiA301::COBID LOW  = 0x0080;
    const CiA301::COBID HIGH = 0x00FF;
    
    enum Code
      {
	PVT_SEQUENCE_COUNTER  = 0x00,
	PVT_CANNOT_BE_STARTED = 0x01,
	PVT_BUFFER_UNDERFLOW  = 0x02,
      };
  }
  
  namespace TIMESTAMP
  {
    const CiA301::COBID COBID = 0x0100;
  }
  
  namespace PDO
  {
    
    namespace COBID
    {
      const CiA301::COBID LOW  = 0x0180;
      const CiA301::COBID HIGH = 0x057F;
    }

    enum SubIndex { COMMUNICATION = 0x01, TRANSMISSION  = 0x02 };

    enum Transmission 
      { 
	SYNC_ACYCLIC   = 0x00,
	SYNC_CYCLIC    = 0x01,   // not really it's a range
	SYNC_RTR       = 0xFC,  
	ASYNC_RTR      = 0xFD,
	ASYNC          = 0xFE
      };
  }
  
  struct SDO : public Object {

    typedef unsigned short   Index; // 2 bytes
    typedef unsigned char SubIndex; // 1 byte
    typedef int               Data; // 4 bytes
    
    // SDO response (node to host)
    static const CiA301::COBID RESPONSE      = 0x580;
    static const CiA301::COBID RESPONSE_LOW  = 0x581;
    static const CiA301::COBID RESPONSE_HIGH = 0x5FF;

    // SDO request (host to node)
    static const CiA301::COBID REQUEST       = 0x600;
    static const CiA301::COBID REQUEST_LOW   = 0x601;
    static const CiA301::COBID REQUEST_HIGH  = 0x680;
    
    enum Command
      {
	INITIATE_READ  = 0x40,
	INITIATE_WRITE = 0x22
      };
    
    SDO( CiA301::SDO::Command   command,
	 CiA301::SDO::Index       index,
	 CiA301::SDO::SubIndex subindex) :
      Object( CiA301::Object::DataField( command, 
					 (index>>0) & 0xFF, 
					 (index>>8) & 0xFF,
					 subindex,
					 0x00, 0x00, 0x00, 0x00 ) ) {}
    SDO( CiA301::SDO::Command   command,
	 CiA301::SDO::Index       index,
	 CiA301::SDO::SubIndex subindex,
	 CiA301::SDO::Data         data) : 
      Object( CiA301::Object::DataField( command, 
					 (index>>0) & 0xFF, 
					 (index>>8) & 0xFF,
					 subindex,
					 (data>> 0) & 0xFF,
					 (data>> 8) & 0xFF,
					 (data>>16) & 0xFF,
					 (data>>24) & 0xFF ) ) {}
  };
  
  struct RPDO2 : public CiA301::SDO {
    RPDO2( CiA301::COBID cobid ) : 
      SDO( CiA301::SDO::INITIATE_WRITE, 
	   0x1401, 
	   CiA301::PDO::COMMUNICATION, 
	   cobid ) {}
    
    RPDO2( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE,  
	   0x1401, 
	   CiA301::PDO::TRANSMISSION, 
	   transmission ){}

    static CiA301::Object Data( int word, int mode ){
      return CiA301::Object( CiA301::Object::DataField( (word>>0) & 0xFF,
							(word>>8) & 0xFF,
							(mode>>0) & 0xFF,
							(mode>>8) & 0xFF ) );
    }

  };

  struct RPDO3 : public CiA301::SDO {
    RPDO3( CiA301::COBID cobid ) : 
      SDO( CiA301::SDO::INITIATE_WRITE, 
	   0x1402, 
	   CiA301::PDO::COMMUNICATION, 
	   cobid ) {}
    
    RPDO3( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE,  
	   0x1402, 
	   CiA301::PDO::TRANSMISSION, 
	   transmission ){}

    static CiA301::Object Data( int word, int pos ){
      return CiA301::Object( CiA301::Object::DataField( (word>>0) & 0xFF,
							(word>>8) & 0xFF,
							(pos>> 0) & 0xFF,
							(pos>> 8) & 0xFF,
							(pos>>16) & 0xFF,
							(pos>>24) & 0xFF ) );
    }

  };
  
  struct RPDO4 : public CiA301::SDO {
    RPDO4( CiA301::COBID cobid ) : 
      SDO( CiA301::SDO::INITIATE_WRITE, 
	   0x1403, 
	   CiA301::PDO::COMMUNICATION, 
	   cobid ) {}
    
    RPDO4( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE,  
	   0x1403, 
	   CiA301::PDO::TRANSMISSION, 
	   transmission ){}

    static CiA301::Object Data( int word, int vel ){
      return CiA301::Object( CiA301::Object::DataField( (word>>0) & 0xFF,
							(word>>8) & 0xFF,
							(vel>> 0) & 0xFF,
							(vel>> 8) & 0xFF,
							(vel>>16) & 0xFF,
							(vel>>24) & 0xFF ) );
    }

  };
  

  struct RPDO5 : public CiA301::SDO {
    RPDO5( CiA301::COBID cobid ) : 
      SDO( CiA301::SDO::INITIATE_WRITE, 
	   0x1404, 
	   CiA301::PDO::COMMUNICATION, 
	   cobid ) {}
    
    RPDO5( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE,  
	   0x1404, 
	   CiA301::PDO::TRANSMISSION, 
	   transmission ){}

    static CiA301::Object Data( int word, int cur ){
      return CiA301::Object( CiA301::Object::DataField( (word>>0) & 0xFF,
							(word>>8) & 0xFF,
							(cur>> 0) & 0xFF,
							(cur>> 8) & 0xFF,
							(cur>>16) & 0xFF,
							(cur>>24) & 0xFF ) );
    }

  };
  

  struct RPDO21 : public CiA301::SDO {
    RPDO21( CiA301::COBID cobid ) :
      SDO( CiA301::SDO::INITIATE_WRITE, 
	   0x1414, 
	   CiA301::PDO::COMMUNICATION, 
	   cobid ) {}
    
    RPDO21( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE,  
	   0x1414, 
	   CiA301::PDO::TRANSMISSION, 
	   transmission ){}
  };
  
  struct RPDO24 : public CiA301::SDO {
    RPDO24( CiA301::COBID cobid ) :
      SDO( CiA301::SDO::INITIATE_WRITE, 
	   0x1417, 
	   CiA301::PDO::COMMUNICATION, 
	   cobid ) {}
    
    RPDO24( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE,  
	   0x1417, 
	   CiA301::PDO::TRANSMISSION, 
	   transmission ){}
  };
  
  
  struct TPDO1 : public CiA301::SDO {
    TPDO1( CiA301::COBID cobid ) :
      SDO( CiA301::SDO::INITIATE_WRITE, 0x1800,
	   CiA301::PDO::COMMUNICATION,  cobid ) {}
    
    TPDO1( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE, 0x1800, 
	   CiA301::PDO::TRANSMISSION,   transmission ){}
  };
  
  struct TPDO2 : public CiA301::SDO {
    TPDO2( CiA301::COBID cobid ) :
      SDO( CiA301::SDO::INITIATE_WRITE, 0x1801,
	   CiA301::PDO::COMMUNICATION,  cobid ) {}
    
    TPDO2( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE, 0x1801, 
	   CiA301::PDO::TRANSMISSION,   transmission ){}
  };

  struct TPDO3 : public CiA301::SDO {
    TPDO3( CiA301::COBID cobid ) :
      SDO( CiA301::SDO::INITIATE_WRITE, 0x1802,
	   CiA301::PDO::COMMUNICATION,  cobid ) {}
    
    TPDO3( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE, 0x1802, 
	   CiA301::PDO::TRANSMISSION,   transmission ){}
  };
  
  struct TPDO4 : public CiA301::SDO {
    TPDO4( CiA301::COBID cobid ) :
      SDO( CiA301::SDO::INITIATE_WRITE, 0x1803,
	   CiA301::PDO::COMMUNICATION,  cobid ) {}
    
    TPDO4( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE, 0x1803, 
	   CiA301::PDO::TRANSMISSION,   transmission ){}
  };
    
  struct TPDO5 : public CiA301::SDO {
    TPDO5( CiA301::COBID cobid ) :
      SDO( CiA301::SDO::INITIATE_WRITE, 0x1804,
	   CiA301::PDO::COMMUNICATION,  cobid ) {}
    
    TPDO5( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE, 0x1804, 
	   CiA301::PDO::TRANSMISSION,   transmission ){}
  };
  
  struct TPDO21 : public CiA301::SDO {
    TPDO21( CiA301::COBID cobid ) :
      SDO( CiA301::SDO::INITIATE_WRITE, 
	   0x1814,
	   CiA301::PDO::COMMUNICATION, 
	   cobid ) {}
    
    TPDO21( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE,  
	   0x1814, 
	   CiA301::PDO::TRANSMISSION, 
	   transmission ){}
  };
  
  struct TPDO24 : public CiA301::SDO {
    TPDO24( CiA301::COBID cobid ) :
      SDO( CiA301::SDO::INITIATE_WRITE, 
	   0x1817,
	   CiA301::PDO::COMMUNICATION, 
	   cobid ) {}
    
    TPDO24( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE,  
	   0x1817, 
	   CiA301::PDO::TRANSMISSION, 
	   transmission ){}
  };
  
  struct TPDO25 : public CiA301::SDO {
    TPDO25( CiA301::COBID cobid ) :
      SDO( CiA301::SDO::INITIATE_WRITE, 
	   0x1818,
	   CiA301::PDO::COMMUNICATION, 
	   cobid ) {}
    
    TPDO25( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE,  
	   0x1818, 
	   CiA301::PDO::TRANSMISSION, 
	   transmission ){}
  };
  
  struct TPDO26 : public CiA301::SDO {
    
    enum Mapping
      { 
	MAPPING_CNT = 0x00,
	MAPPING_1   = 0x01, 
	MAPPING_2   = 0x02, 
	MAPPING_3   = 0x03, 
	MAPPING_4   = 0x04,
	MAPPING_5   = 0x05,
      };
    
    TPDO26( CiA301::COBID cobid ) :
      SDO( CiA301::SDO::INITIATE_WRITE, 
	   0x1819,
	   CiA301::PDO::COMMUNICATION, 
	   cobid ) {}
    
    TPDO26( CiA301::PDO::Transmission transmission ) : 
      SDO( CiA301::SDO::INITIATE_WRITE,
	   0x1819,
	   CiA301::PDO::TRANSMISSION, 
	   transmission ){}
    
    TPDO26( CiA301::TPDO26::Mapping mapping,  CiA301::TPDO26::Data data ) :
      SDO( CiA301::SDO::INITIATE_WRITE, 0x1A19, mapping, data ){}
    
  };
   
}

#endif

