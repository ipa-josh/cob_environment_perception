#pragma once

#include <boost/asio/ip/tcp.hpp>


//! interfaces and implementations of cob_3d_experience_mapping
namespace cob_3d_experience_mapping{
//! interfaces and implementations needed to serialize map with boost
namespace serialization {

class PackedStream: public std::streambuf
{
    std::iostream &ios_;
    uint32_t pos_, size_;
    std::vector<char> buffer_, read_buffer_;
    bool close_after_packet_;
    
    void new_frame() {
		buffer_.resize(4);
		pos_ = 0;
	}
	
	inline uint32_t get_size() const {
		return size_;
	} 
    
public:
    PackedStream(std::iostream &ios) :
        ios_(ios), pos_(4), size_(0), close_after_packet_(false)
    {
        setg(0, 0, 0);
		new_frame();
    }

    virtual ~PackedStream()
    {
        sync();
    }
    
    void set_close_after_packet(const bool b) {size_ = pos_ = 0; close_after_packet_=b;}

    virtual std::streambuf::int_type underflow()
    {
		if(pos_>=4) {
			if(close_after_packet_) return traits_type::eof();
			size_ = pos_ = 0;
		}
		
		DBG_PRINTF("underflow1\n");
		if(pos_<4) {
			ios_.read((char*)&size_, 4-pos_);
			pos_=4;
			read_buffer_.resize(size_-4);
		}
		
		DBG_PRINTF("underflow2 %d %d\n", pos_, get_size());
		if(pos_>=get_size()) {
			size_ = pos_ = 0;
			return traits_type::eof();
		}
		
		ios_.read(&read_buffer_[0], size_-4);
		pos_+=read_buffer_.size();
        setg(&read_buffer_[0], &read_buffer_[0], &read_buffer_[0]+read_buffer_.size());
        
        return traits_type::to_int_type(*this->gptr());
    }

    virtual std::streambuf::int_type overflow(std::streambuf::int_type value)
    {
		buffer_.push_back(value);
        return traits_type::not_eof(value);
    };

    virtual int sync()
    {
		ios_.flush();
		return ios_.sync();
    }
    
    void finish() {
		*((uint32_t*)&buffer_[0]) = (uint32_t)buffer_.size();
		ios_.write(&buffer_[0], buffer_.size());
		ios_.flush();
		new_frame();
	}
};

class IOPackedStream: public std::iostream
{
public:
    IOPackedStream(std::iostream &stream) :
        std::iostream(new PackedStream(stream)) {}

    virtual ~IOPackedStream()
    {
        delete rdbuf();
    }
    
    void finish() {
		((PackedStream*)rdbuf())->finish();
	}
	
    void set_close_after_packet(const bool b) {((PackedStream*)rdbuf())->set_close_after_packet(b);}
};

	
	template<class _TClientId, class _TID>
	struct NetworkHeader {
		typedef _TClientId TClientId;
		typedef _TID TID;
		
		TClientId client_;
		TID ts_states_;
		TID ts_fts_;
		TID ts_trans_;
		bool compression_;
		
		NetworkHeader():
			client_((TClientId)-1), ts_states_((TID)-1), ts_fts_((TID)-1), ts_trans_((TID)-1), compression_(true)
		{ }
		
		NetworkHeader(const TClientId &client, const TID &ts_states, const TID &ts_fts, const TID &ts_trans, const bool compression = true):
			client_(client), ts_states_(ts_states), ts_fts_(ts_fts), ts_trans_(ts_trans), compression_(compression)
		{ }
		
		UNIVERSAL_SERIALIZE()
		{
		   assert(version==CURRENT_SERIALIZATION_VERSION);
		   
		   ar & UNIVERSAL_SERIALIZATION_NVP(client_);
		   ar & UNIVERSAL_SERIALIZATION_NVP(ts_states_);
		   ar & UNIVERSAL_SERIALIZATION_NVP(ts_fts_);
		   ar & UNIVERSAL_SERIALIZATION_NVP(ts_trans_);
		   ar & UNIVERSAL_SERIALIZATION_NVP(compression_);
		}
		   
	};
	
template<class TArchiveIn, class TArchiveOut, class TContent>
void sync_content_client(TContent &s, const char * addr, const char * port, const int timeout_secs=120){
    // make an archive
    boost::asio::ip::tcp::iostream net_stream;
    
    //set timeout
    net_stream.expires_from_now(boost::posix_time::seconds(timeout_secs));
    
    //connect to server and upload our data
    net_stream.connect(addr, port);
    
    assert(net_stream.good());
    
    IOPackedStream stream(net_stream);
    
    //send request header without compression
    typename TContent::TNetworkHeader request_header = s.get_network_header();
	export_content<TArchiveOut>(request_header, stream);
	stream.finish();
	
	IOPackedStream *packed_stream = dynamic_cast<IOPackedStream*>(&stream);
		
    if(packed_stream && request_header.compression_) {
		packed_stream->set_close_after_packet(true);
		export_content_compr<TArchiveOut>(s, stream);
	}
    else
		export_content<TArchiveOut>(s, stream);
	stream.finish();
		
	DBG_PRINTF("sent data\n");
		
    //retrieve request header without compression
    typename TContent::TNetworkHeader response_header;
	import_content<TArchiveIn>(response_header, stream);
	s.set_network_header(response_header);
	
	DBG_PRINTF("got header\n");
	
    if(packed_stream && response_header.compression_) {
		packed_stream->set_close_after_packet(true);
		import_content_compr<TArchiveIn>(s, stream);
	}
	else
		import_content<TArchiveIn>(s, stream);
		
	DBG_PRINTF("got data\n");
}
	
template<class TArchiveIn, class TArchiveOut, class TContent>
void sync_content_server_import_header(TContent &s, std::iostream &stream){    
    assert(stream.good());
    
    //retrieve request header without compression
    typename TContent::TNetworkHeader response_header;
	import_content<TArchiveIn>(response_header, stream);
	s.set_network_header(response_header);
}
	
template<class TArchiveIn, class TArchiveOut, class TContent>
void sync_content_server_import(TContent &s, std::iostream &stream){    
    assert(stream.good());
	IOPackedStream *packed_stream = dynamic_cast<IOPackedStream*>(&stream);
    
	DBG_PRINTF("waiting for data\n");
	
	try {
	
    if(packed_stream && s.get_network_header().compression_) {
		packed_stream->set_close_after_packet(true);
		import_content_compr<TArchiveIn>(s, stream);
	}
	else
		import_content<TArchiveIn>(s, stream);
		
		DBG_PRINTF("got data\n");
	}
	catch(...) {
		ERROR_PRINTF("ERROR: deserialization failed");
	}
}
	
template<class TArchiveIn, class TArchiveOut, class TContent>
void sync_content_server_export(TContent &s, std::iostream &stream){    
    assert(stream.good());
	IOPackedStream *packed_stream = dynamic_cast<IOPackedStream*>(&stream);
    
    //send request header without compression
    typename TContent::TNetworkHeader request_header = s.get_network_header();
	export_content<TArchiveOut>(request_header, stream);
	if(packed_stream) packed_stream->finish();
	
	DBG_PRINTF("sent header\n");
		
    if(packed_stream && request_header.compression_) {
		packed_stream->set_close_after_packet(true);
		export_content_compr<TArchiveOut>(s, stream);
	}
    else
		export_content<TArchiveOut>(s, stream);
	if(packed_stream) packed_stream->finish();
		
	DBG_PRINTF("sent data\n");
}

}
}

