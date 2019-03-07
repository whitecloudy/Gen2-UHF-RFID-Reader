/* -*- c++ -*- */
/*
 * Copyright 2015 <Nikos Kargas (nkargas@isc.tuc.gr)>.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_RFID_TAG_DECODER_IMPL_H
#define INCLUDED_RFID_TAG_DECODER_IMPL_H

#include <rfid/tag_decoder.h>
#include <vector>
#include "rfid/global_vars.h"
#include <time.h>
#include <numeric>
#include <fstream>
namespace gr {
  namespace rfid {

    class tag_decoder_impl : public tag_decoder
    {
      private:

        float n_samples_TAG_BIT;
        int s_rate;
        std::vector<float> pulse_bit;
        float T_global;
        gr_complex h_est;
        char * char_bits;
        FILE *preamble_fp;
        int success_count;

        int check_crc(char * bits, int num_bits);

        //tag_decoder_impl.cc
        const gr_complex* in;
        std::vector<float> norm_in;
        std::vector<gr_complex> sample;
        std::vector<float> norm_sample;
        int size;

        // tag_decoder_decoder.cc
        std::vector<int> center;
        std::vector<int> clustered_id;

        int cut_noise_sample(std::vector<float> in, const int total_len, const int data_len);
        int tag_sync(std::vector<float> in, int size);
        int determine_first_mask_level(std::vector<float> in, int index);
        int decode_single_bit(std::vector<float> in, int index, int mask_level, float* ret_corr);
        std::vector<float> tag_detection(std::vector<float> in, int index, int n_expected_bit);

        // tag_decoder_clustering.cc
        double IQ_distance(const gr_complex p1, const gr_complex p2);
        void center_identification();
        float norm_2dim_gaussian_pdf(const gr_complex value, const gr_complex mean, const float standard_deviation);
        float pd_i_k(const int i, const int k);
        float pt_i_k(const int i, const int k);
        int max_id_pcluster_i(const int i);
        float max_value_pcluster_i(const int i);
        void sample_clustering();
        void print_cluster_sample(const std::string filename);

        // tag_decoder_OFG.cc
        typedef struct _OFG_node
        {
          int id;
          int layer;
          int* state;
          std::vector<int> link;
        } OFG_node;

        void count_flip(int** flip_info, const std::vector<int> clustered_idx, int size);
        int check_odd_cycle_OFG(OFG_node* OFG, int start, int compare, int check, std::vector<int> stack);
        void construct_OFG(OFG_node* OFG, int** flip_info, int size, int n_tag);
        void determine_OFG_state(OFG_node* OFG, int size, int n_tag);
        void extract_parallel_sample(std::vector<int>* extracted_sample, const std::vector<int> clustered_idx, const OFG_node* OFG, int n_tag);

      public:
        tag_decoder_impl(int sample_rate, std::vector<int> output_sizes);
        ~tag_decoder_impl();

        void forecast (int noutput_items, gr_vector_int &ninput_items_required);

        int general_work(int noutput_items,
            gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
    };

  } // namespace rfid
} // namespace gr

#endif /* INCLUDED_RFID_TAG_DECODER_IMPL_H */
