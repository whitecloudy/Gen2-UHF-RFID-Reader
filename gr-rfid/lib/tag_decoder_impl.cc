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

      return gnuradio::get_initial_sptr
      (new tag_decoder_impl(sample_rate,output_sizes));
    }

    tag_decoder_impl::tag_decoder_impl(int sample_rate, std::vector<int> output_sizes)
    : gr::block("tag_decoder", gr::io_signature::make(1, 1, sizeof(gr_complex)), gr::io_signature::makev(2, 2, output_sizes)), s_rate(sample_rate)
    {
      char_bits = new char[128];
      n_samples_TAG_BIT = TAG_BIT_D * s_rate / pow(10, 6);
    }

    tag_decoder_impl::~tag_decoder_impl()
    {

    }

    void tag_decoder_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int tag_decoder_impl::general_work (int noutput_items, gr_vector_int& ninput_items, gr_vector_const_void_star& input_items, gr_vector_void_star& output_items)
    {
      float* out = (float*)output_items[0];
      int consumed = 0;

      if(ninput_items[0] >= reader_state->n_samples_to_ungate)
      {
        log.open(log_file_path, std::ios::app);

        int mode = -1;  // 0:RN16, 1:EPC
        if(reader_state->decoder_status == DECODER_DECODE_RN16) mode = 0;
        else if(reader_state->decoder_status == DECODER_DECODE_EPC) mode = 1;

        sample_information ys((gr_complex*)input_items[0], ninput_items[0], n_samples_TAG_BIT, mode);

        #ifdef DEBUG_MESSAGE_SAMPLE
        std::string debug_file_path = debug_folder_path + std::to_string(reader_state->reader_stats.cur_inventory_round) + "_" + std::to_string(reader_state->reader_stats.cur_slot_number) + "_";
        if(mode == 0) debug_file_path = debug_file_path + "RN16_sample";
        else if(mode == 1) debug_file_path = debug_file_path + "EPC_sample";
        debug_sample.open(debug_file_path, std::ios::app);
        print_sample(&ys);
        debug_sample.close();
        #endif

        int n_tag = clustering_sample(&ys, mode);

        if(n_tag == 1)
        {

        }
        else if(n_tag > 1)
        {
          #ifdef DEBUG_MESSAGE_OFG
          std::string debug_file_path = debug_folder_path + std::to_string(reader_state->reader_stats.cur_inventory_round) + "_" + std::to_string(reader_state->reader_stats.cur_slot_number) + "_";
          if(mode == 0) debug_file_path = debug_file_path + "RN16_OFG";
          else if(mode == 1) debug_file_path = debug_file_path + "EPC_OFG";
          debug_OFG.open(debug_file_path, std::ios::app);
          #endif

          extract_parallel_sample(&ys);

          #ifdef DEBUG_MESSAGE_OFG
          debug_OFG.close();
          #endif
        }

        // process for GNU RADIO
        produce(1, ninput_items[0]);
        consumed = reader_state->n_samples_to_ungate;

        log.close();
      }

      consume_each(consumed);
      return WORK_CALLED_PRODUCE;
    }

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

/*      // Processing only after n_samples_to_ungate are available and we need to decode an RN16
      if(reader_state->decoder_status == DECODER_DECODE_RN16 && ninput_items[0] >= reader_state->n_samples_to_ungate)
      {


        if(n_tag > 1)
        {

          int** flip_info = new int*[center.size()];
          for(int i=0 ; i<center.size() ; i++)
            flip_info[i] = new int[center.size()];
          count_flip(flip_info, clustered_idx, center.size());

          OFG_node* OFG = new OFG_node[center.size()];
          construct_OFG(OFG, flip_info, center.size(), n_tag);
          for(int i=0 ; i<center.size() ; i++)
          {
            OFG[i].layer = n_tag + 1;
            OFG[i].state = new int[n_tag];
            for(int j=0 ; j<n_tag ; j++)
              OFG[i].state[j] = -1;
          }
          determine_OFG_state(OFG, center.size(), n_tag);

          std::vector<int>* extracted_sample = new std::vector<int>[n_tag];
          extract_parallel_sample(extracted_sample, clustered_idx, OFG, n_tag);

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

          for(int i=0 ; i<center.size() ; i++)
            delete flip_info[i];
          delete flip_info;

          for(int i=0 ; i<center.size() ; i++)
            delete OFG[i].state;
          delete[] OFG;

          delete[] extracted_sample;
        }
        else
        {
          // detect preamble
          int RN16_index = tag_sync(norm_in, ninput_items[0]);  //find where the tag data bits start

          // decode RN16
          if(RN16_index != -1)
          {
            log.open("debug_message", std::ios::app);
            log << "│ Preamble detected!" << std::endl;
            log.close();
            std::vector<float> RN16_bits = tag_detection(norm_in, RN16_index, RN16_BITS-1);  // RN16_BITS includes one dummy bit

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
          }
          else  // fail to detect preamble
          {
            log.open("debug_message", std::ios::app);
            log << "│ Preamble detection fail.." << std::endl;
            std::cout << "\t\t\t\t\tPreamble FAIL!!";

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
        }*/
/*

      }
*/
      // Processing only after n_samples_to_ungate are available and we need to decode an EPC
      /*else if (reader_state->decoder_status == DECODER_DECODE_EPC && ninput_items[0] >= reader_state->n_samples_to_ungate )
      {
        #ifdef DEBUG_MESSAGE
        {
          debug.open((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str(), std::ios::app);
          debug << "n_samples_to_ungate= " << reader_state->n_samples_to_ungate << ", ninput_items[0]= " << ninput_items[0] << std::endl;
          debug << "\t\t\t\t\t** samples from gate **" << std::endl;
          for(int i=0 ; i<ninput_items[0] ; i++)
            debug << norm_in[i] << " ";
          debug << std::endl << "\t\t\t\t\t** samples from gate **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();

          debug.open((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)+"_iq").c_str(), std::ios::app);
          debug << "n_samples_to_ungate= " << reader_state->n_samples_to_ungate << ", ninput_items[0]= " << ninput_items[0] << std::endl;
          debug << "\t\t\t\t\t** samples from gate (I) **" << std::endl;
          for(int i=0 ; i<ninput_items[0] ; i++)
            debug << in[i].real() << " ";
          debug << std::endl << "\t\t\t\t\t** samples from gate **" << std::endl << std::endl << std::endl << std::endl;
          debug << "\t\t\t\t\t** samples from gate (Q) **" << std::endl;
          for(int i=0 ; i<ninput_items[0] ; i++)
            debug << in[i].imag() << " ";
          debug << std::endl << "\t\t\t\t\t** samples from gate **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();
        }
        #endif

        // detect preamble
        int EPC_index = tag_sync(norm_in, ninput_items[0]);
        #ifdef DEBUG_MESSAGE
        {
          debug.open((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str(), std::ios::app);
          debug << "\t\t\t\t\t** EPC samples **" << std::endl;
          for(int i=0 ; i<n_samples_TAG_BIT*(EPC_BITS-1) ; i++)
            debug << norm_in[EPC_index+i] << " ";
          debug << std::endl << "\t\t\t\t\t** EPC samples **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();

          debug.open((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)+"_iq").c_str(), std::ios::app);
          debug << "\t\t\t\t\t** EPC samples (I) **" << std::endl;
          for(int i=0 ; i<n_samples_TAG_BIT*(EPC_BITS-1) ; i++)
            debug << in[EPC_index+i].real() << " ";
          debug << std::endl << "\t\t\t\t\t** EPC samples **" << std::endl << std::endl << std::endl << std::endl;
          debug << "\t\t\t\t\t** EPC samples (Q) **" << std::endl;
          for(int i=0 ; i<n_samples_TAG_BIT*(EPC_BITS-1) ; i++)
            debug << in[EPC_index+i].imag() << " ";
          debug << std::endl << "\t\t\t\t\t** EPC samples **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();
        }
        #endif

        // process for GNU RADIO
        int written_sync = 0;
        for(int j=0 ; j<ninput_items[0] ; j++)
          written_sync++;
        produce(1, written_sync);

        // decode EPC
        if(EPC_index != -1)
        {
          log.open("debug_message", std::ios::app);
          log << "│ Preamble detected!" << std::endl;
          log.close();
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
            log.open("debug_message", std::ios::app);
            log << "CRC check success! Tag ID= " << tag_id << std::endl;
            log.close();
            std::cout << "\t\t\t\t\t\t\t\t\t\tTag ID= " << tag_id;
            reader_state->reader_stats.n_epc_correct+=1;

            // Save part of Tag's EPC message (EPC[104:111] in decimal) + number of reads
            std::map<int,int>::iterator it = reader_state->reader_stats.tag_reads.find(tag_id);
            if ( it != reader_state->reader_stats.tag_reads.end())
              it->second ++;
            else
              reader_state->reader_stats.tag_reads[tag_id]=1;
          }
          else
          {
            log.open("debug_message", std::ios::app);
            log << "│ CRC check fail.." << std::endl;
            log.close();
            std::cout << "\t\t\t\t\tCRC FAIL!!";
          }
        }
        else
        {
          log.open("debug_message", std::ios::app);
          log << "│ Preamble detection fail.." << std::endl;
          log.close();
          std::cout << "\t\t\t\t\tPreamble FAIL!!";
        }

        // After EPC message send a query rep or query
        log.open("debug_message", std::ios::app);
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

        // process for GNU RADIO
        //consumed = reader_state->n_samples_to_ungate;
      }*/

    int tag_decoder_impl::clustering_sample(sample_information* ys, int mode)
    {
      #ifdef DEBUG_MESSAGE_CLUSTER
      std::string debug_file_path = debug_folder_path + std::to_string(reader_state->reader_stats.cur_inventory_round) + "_" + std::to_string(reader_state->reader_stats.cur_slot_number) + "_";
      if(mode == 0) debug_file_path = debug_file_path + "RN16_cluster";
      debug_cluster.open(debug_file_path, std::ios::app);
      #endif

      center_identification(ys);
      sample_clustering(ys);

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

      #ifdef DEBUG_MESSAGE_CLUSTER
      debug_cluster.close();
      #endif

      return ys->n_tag();
    }

    void tag_decoder_impl::extract_parallel_sample(sample_information* ys)
    {
      count_flip(ys);
      construct_OFG(ys);
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
  } /* namespace rfid */
} /* namespace gr */
