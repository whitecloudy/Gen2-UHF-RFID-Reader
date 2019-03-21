/* -*- c++ -*- */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/prefs.h>
#include <gnuradio/math.h>
#include <cmath>
#include <sys/time.h>
#include "tag_decoder_impl.h"

namespace gr
{
  namespace rfid
  {
    tag_decoder::sptr
    tag_decoder::make(int sample_rate)
    {
      std::vector<int> output_sizes;
      output_sizes.push_back(sizeof(float));
      output_sizes.push_back(sizeof(gr_complex));

      return gnuradio::get_initial_sptr(new tag_decoder_impl(sample_rate,output_sizes));
    }

    tag_decoder_impl::tag_decoder_impl(int sample_rate, std::vector<int> output_sizes)
    : gr::block("tag_decoder", gr::io_signature::make(1, 1, sizeof(gr_complex)), gr::io_signature::makev(2, 2, output_sizes)), s_rate(sample_rate)
    {
      char_bits = new char[128];
      n_samples_TAG_BIT = TPRI_D * s_rate / pow(10,6);
      //GR_LOG_INFO(d_logger, "Number of samples of Tag bit : "<< n_samples_TAG_BIT);
    }

    tag_decoder_impl::~tag_decoder_impl(){}

    void tag_decoder_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int tag_decoder_impl::general_work(int noutput_items, gr_vector_int& ninput_items, gr_vector_const_void_star& input_items, gr_vector_void_star& output_items)
    {
      float* out = (float*)output_items[0];
      int consumed = 0;

      if(ninput_items[0] >= reader_state->n_samples_to_ungate)
      {
        int mode = -1;  // 0:RN16, 1:EPC
        if(reader_state->decoder_status == DECODER_DECODE_RN16) mode = 0;
        else if(reader_state->decoder_status == DECODER_DECODE_EPC) mode = 1;
        open_debug_ofstream(mode);

        sample_information ys((gr_complex*)input_items[0], ninput_items[0], n_samples_TAG_BIT, mode);

        #ifdef DEBUG_MESSAGE_SAMPLE
        print_sample(&ys);
        #endif

        int n_tag = clustering_sample(&ys, mode);
        if(n_tag == 1)
        {
          // detect preamble
          int data_index = tag_sync(&ys);  // find where the tag data bits start

          // process for GNU RADIO
          int written_sync = 0;
          for(int j=0 ; j<ninput_items[0] ; j++)
            written_sync++;
          produce(1, written_sync);

          if(data_index == -1) goto_next_slot();
          log << "│ Preamble detected!" << std::endl;

          if(mode == 0) decode_RN16(&ys, data_index);
          else if(mode == 1) decode_EPC(&ys, data_index);
        }
        else if(n_tag > 1)
        {
          extract_parallel_sample(&ys);
        }

        // process for GNU RADIO
        produce(1, ninput_items[0]);
        consumed = reader_state->n_samples_to_ungate;
      }

      consume_each(consumed);
      return WORK_CALLED_PRODUCE;
    }


/*      // Processing only after n_samples_to_ungate are available and we need to decode an RN16
      if(reader_state->decoder_status == DECODER_DECODE_RN16 && ninput_items[0] >= reader_state->n_samples_to_ungate)
      {


        if(n_tag > 1)
        {


          int i;
          for(i=0 ; i<n_tag ; i++)
          {
            std::vector<float> float_sample;
            for(int j=0 ; j<extracted_sample[i].size() ; j++)
              float_sample.push_back(extracted_sample[i][j]);

            // detect preamble
            int RN16_index = tag_sync(float_sample, data_idx[1]);  //find where the tag data bits start

            // decode RN16
            if(RN16_index != -1)
            {
              log.open("debug_message", std::ios::app);
              log << "│ Preamble detected!" << std::endl;
              log.close();
              std::vector<float> RN16_bits = tag_detection(float_sample, RN16_index, RN16_BITS-1);  // RN16_BITS includes one dummy bit

              // write RN16_bits to the next block
              log.open("debug_message", std::ios::app);
              log << "│ RN16=";
              int written = 0;
              for(int i=0 ; i<RN16_bits.size() ; i++)
              {
                if(i % 4 == 0) std::cout << " ";
                log << RN16_bits[i];
                out[written++] = RN16_bits[i];
              }
              produce(0, written);

              // go to the next state
              log << std::endl << "├──────────────────────────────────────────────────" << std::endl;
              log.close();
              std::cout << "RN16 decoded | ";
              reader_state->gen2_logic_status = SEND_ACK;
              break;
            }
            else  // fail to detect preamble
            {
              log.open("debug_message", std::ios::app);
              log << "│ Preamble detection fail.." << std::endl;
              std::cout << "\t\t\t\t\tPreamble FAIL!!";
            }
          }
          if(i == n_tag)
          {
            reader_state->reader_stats.cur_slot_number++;
            if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
            {
              reader_state->reader_stats.cur_inventory_round ++;
              reader_state->reader_stats.cur_slot_number = 1;

              log << "└──────────────────────────────────────────────────" << std::endl;
              if(reader_state->reader_stats.cur_inventory_round > MAX_NUM_QUERIES)
              {
                reader_state->reader_stats.cur_inventory_round--;
                reader_state-> status = TERMINATED;
                reader_state->decoder_status = DECODER_TERMINATED;
              }
              else
                reader_state->gen2_logic_status = SEND_QUERY;
            }
            else
            {
              log << "├──────────────────────────────────────────────────" << std::endl;
              reader_state->gen2_logic_status = SEND_QUERY_REP;
            }
            log.close();
          }
*/




    int tag_decoder_impl::clustering_sample(sample_information* ys, int mode)
    {
      center_identification(ys);
      sample_clustering(ys);

      #ifdef DEBUG_MESSAGE_CLUSTER
      print_cluster_sample(ys);
      #endif

      cut_tiny_center(ys);

      #ifdef DEBUG_MESSAGE_CLUSTER
      print_cluster_sample(ys);
      #endif

      if(ys->center_size() == 1 || !is_power_of_2(ys))
      {
        clustering_error_detection(ys);
        ys->clear_cluster();
        sample_clustering(ys);

        #ifdef DEBUG_MESSAGE_CLUSTER
        print_cluster_sample(ys);
        #endif
      }
      calc_n_tag(ys);

      return ys->n_tag();
    }

    bool tag_decoder_impl::extract_parallel_sample(sample_information* ys)
    {
      count_flip(ys);
      if(!construct_OFG(ys)) return false;
      determine_OFG_state(ys);
      ys->set_binary_sample();

      #ifdef DEBUG_MESSAGE_OFG
      print_binary_sample(ys);
      #endif

      return true;
    }

    void tag_decoder_impl::decode_RN16(sample_information* ys, int data_index)
    {
      std::vector<float> RN16_bits = tag_detection(ys, data_index, RN16_BITS-1);  // RN16_BITS includes one dummy bit

      // write RN16_bits to the next block
      log << "│ RN16=";
      int written = 0;
      for(int i=0 ; i<RN16_bits.size() ; i++)
      {
        if(i % 4 == 0) std::cout << " ";
        log << RN16_bits[i];
        out[written++] = RN16_bits[i];
      }
      produce(0, written);

      // go to the next state
      log << std::endl << "├──────────────────────────────────────────────────" << std::endl;
      std::cout << "RN16 decoded | ";
      reader_state->gen2_logic_status = SEND_ACK;
    }

    void tag_decoder_impl::decode_EPC(sample_information* ys, int data_index)
    {
      std::vector<float> EPC_bits = tag_detection(norm_in, EPC_index, EPC_BITS-1);  // EPC_BITS includes one dummy bit

      // convert EPC_bits from float to char in order to use Buettner's function
      log.open("debug_message", std::ios::app);
      log << "│ EPC=";
      for(int i=0 ; i<EPC_bits.size() ; i++)
      {
        if(i % 4 == 0) log << " ";
        log << EPC_bits[i];
        char_bits[i] = EPC_bits[i] + '0';
        if(i % 16 == 15) log << std::endl << "│     ";
      }
      log.close();

      // check CRC
      if(check_crc(char_bits, 128) == 1) // success to decode EPC
      {
        // calculate tag_id
        int tag_id = 0;
        for(int i=0 ; i<8 ; i++)
          tag_id += std::pow(2, 7-i) * EPC_bits[104+i];

        //GR_LOG_INFO(d_debug_logger, "EPC CORRECTLY DECODED, TAG ID : " << tag_id);
        log << "CRC check success! Tag ID= " << tag_id << std::endl;
        std::cout << "\t\t\t\t\t\t\t\t\t\tTag ID= " << tag_id;
        reader_state->reader_stats.n_epc_correct++;

        // Save part of Tag's EPC message (EPC[104:111] in decimal) + number of reads
        std::map<int,int>::iterator it = reader_state->reader_stats.tag_reads.find(tag_id);
        if (it != reader_state->reader_stats.tag_reads.end()) it->second ++;
        else reader_state->reader_stats.tag_reads[tag_id]=1;
      }
      else
      {
        log << "│ CRC check fail.." << std::endl;
        std::cout << "\t\t\t\t\tCRC FAIL!!";
      }

      goto_next_slot();
    }

    void tag_decoder_impl::goto_next_slot(void)
    {
      reader_state->reader_stats.cur_slot_number++;
      if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
      {
        reader_state->reader_stats.cur_inventory_round ++;
        reader_state->reader_stats.cur_slot_number = 1;

        log << "└──────────────────────────────────────────────────" << std::endl;
        if(reader_state->reader_stats.cur_inventory_round > MAX_NUM_QUERIES)
        {
          reader_state->reader_stats.cur_inventory_round--;
          reader_state->status = TERMINATED;
          reader_state->decoder_status = DECODER_TERMINATED;
        }
        else reader_state->gen2_logic_status = SEND_QUERY;
      }
      else
      {
        log << "├──────────────────────────────────────────────────" << std::endl;
        reader_state->gen2_logic_status = SEND_QUERY_REP;
      }
    }

    /* Function adapted from https://www.cgran.org/wiki/Gen2 */
    int tag_decoder_impl::check_crc(char * bits, int num_bits)
    {
      register unsigned short i, j;
      register unsigned short crc_16, rcvd_crc;
      unsigned char * data;
      int num_bytes = num_bits / 8;
      data = (unsigned char* )malloc(num_bytes );
      int mask;

      for(i = 0; i < num_bytes; i++)
      {
        mask = 0x80;
        data[i] = 0;
        for(j = 0; j < 8; j++)
        {
          if (bits[(i * 8) + j] == '1'){
            data[i] = data[i] | mask;
          }
          mask = mask >> 1;
        }
      }
      rcvd_crc = (data[num_bytes - 2] << 8) + data[num_bytes -1];

      crc_16 = 0xFFFF;
      for (i=0; i < num_bytes - 2; i++)
      {
        crc_16^=data[i] << 8;
        for (j=0;j<8;j++)
        {
          if (crc_16&0x8000)
          {
            crc_16 <<= 1;
            crc_16 ^= 0x1021;
          }
          else
          crc_16 <<= 1;
        }
      }
      crc_16 = ~crc_16;

      if(rcvd_crc != crc_16)
      return -1;
      else
      return 1;
    }

    void tag_decoder_impl::open_debug_ofstream(int mode)
    {
      std::string debug_file_path_prefix = debug_folder_path + std::to_string(reader_state->reader_stats.cur_inventory_round) + "_" + std::to_string(reader_state->reader_stats.cur_slot_number) + "_";
      std::string debug_file_path;
      log.open(log_file_path, std::ios::app);

      #ifdef DEBUG_MESSAGE_SAMPLE
      debug_file_path.clear();
      if(mode == 0) debug_file_path = debug_file_path_prefix + "RN16_sample";
      else if(mode == 1) debug_file_path = debug_file_path_prefix + "EPC_sample";
      debug_sample.open(debug_file_path, std::ios::app);
      #endif

      #ifdef DEBUG_MESSAGE_DECODER
      debug_file_path.clear();
      if(mode == 0) debug_file_path = debug_file_path_prefix + "RN16_decoder";
      else if(mode == 1) debug_file_path = debug_file_path_prefix + "EPC_decoder";
      debug_decoder.open(debug_file_path, std::ios::app);
      #endif

      #ifdef DEBUG_MESSAGE_CLUSTER
      debug_file_path.clear();
      if(mode == 0) debug_file_path = debug_file_path_prefix + "RN16_cluster";
      else if(mode == 1) debug_file_path = debug_file_path_prefix + "EPC_cluster";
      debug_cluster.open(debug_file_path, std::ios::app);
      #endif

      #ifdef DEBUG_MESSAGE_OFG
      debug_file_path.clear();
      if(mode == 0) debug_file_path = debug_file_path_prefix + "RN16_OFG";
      else if(mode == 1) debug_file_path = debug_file_path_prefix + "EPC_OFG";
      debug_OFG.open(debug_file_path, std::ios::app);
      #endif
    }

    void tag_decoder_impl::close_debug_ofstream(void)
    {
      log.close();

      #ifdef DEBUG_MESSAGE_SAMPLE
      debug_sample.close();
      #endif

      #ifdef DEBUG_MESSAGE_DECODER
      debug_decoder.close();
      #endif

      #ifdef DEBUG_MESSAGE_CLUSTER
      debug_cluster.close();
      #endif

      #ifdef DEBUG_MESSAGE_OFG
      debug_OFG.close();
      #endif
    }

    #ifdef DEBUG_MESSAGE_SAMPLE
    void tag_decoder_impl::print_sample(sample_information* ys)
    {
      debug_sample << "\t\t\t\t\t** I **" << std::endl;
      for(int i=0 ; i<ys->total_size() ; i++)
        debug_sample << ys->in(i).real() << " ";
      debug_sample << std::endl;
      debug_sample << "\t\t\t\t\t** Q **" << std::endl;
      for(int i=0 ; i<ys->total_size() ; i++)
        debug_sample << ys->in(i).imag() << " ";
      debug_sample << std::endl;
      debug_sample << "\t\t\t\t\t** norm **" << std::endl;
      for(int i=0 ; i<ys->total_size() ; i++)
        debug_sample << ys->norm_in(i) << " ";
    }
    #endif
  } /* namespace rfid */
} /* namespace gr */
