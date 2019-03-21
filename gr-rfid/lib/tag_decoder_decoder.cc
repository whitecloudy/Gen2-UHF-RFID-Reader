/* -*- c++ -*- */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "tag_decoder_impl.h"

#define SHIFT_SIZE 3

namespace gr
{
  namespace rfid
  {
    int tag_decoder_impl::tag_sync(sample_information* ys)
    // This method searches the preamble and returns the start index of the tag data.
    // If the correlation value exceeds the threshold, it returns the start index of the tag data.
    // Else, it returns -1.
    // Threshold is an experimental value, so you might change this value within your environment.
    {
      int win_size = n_samples_TAG_BIT * TAG_PREAMBLE_BITS;
      float threshold = n_samples_TAG_BIT * 4;  // threshold verifing correlation value

      float max_corr = 0.0f;
      int max_index = 0;

      // compare all samples with sliding
      for(int i=0 ; i<ys->size()-win_size ; i++)  // i: start point
      {
        // calculate average_amp (threshold)
        float average_amp = 0.0f;
        for(int j=0 ; j<win_size ; j++)
          average_amp += ys->norm_sample(i+j);
        average_amp /= win_size;

        // calculate normalize_factor
        float standard_deviation = 0.0f;
        for(int j=0 ; j<win_size ; j++)
          standard_deviation += pow(ys->norm_sample(i+j) - average_amp, 2);
        standard_deviation /= win_size;
        standard_deviation = sqrt(standard_deviation);

        // calculate correlation value
        float corr_candidates[2] = {0.0f};
        for(int j=0 ; j<2*TAG_PREAMBLE_BITS ; j++)  // j: half_bit index of TAG_PREAMBLE
        {
          for(int k=0 ; k<(n_samples_TAG_BIT/2.0) ; k++)
          {
            for(int m=0 ; m<2 ; m++)  // m: index of TAG_PREAMBLE type
                corr_candidates[m] += TAG_PREAMBLE[m][j] * ((ys->norm_sample(i + j*(int)(n_samples_TAG_BIT/2.0) + k) - average_amp) / standard_deviation);
          }
        }

        // get max correlation value for ith start point
        float corr = 0.0f;
        for(int j=0 ; j<2 ; j++)
          if(corr_candidates[j] > corr) corr = corr_candidates[j];

        // compare with current max correlation value
        if(corr > max_corr)
        {
          max_corr = corr;
          max_index = i;
        }
      }

      #ifdef DEBUG_MESSAGE_DECODER
      debug_decoder << "\t\t\t\t\t** preamble sample **" << std::endl;
      debug_decoder << "threshold= " << threshold << "\tcorr= " << max_corr << "\tdata_index= " << max_index << std::endl;
      for(int i=0 ; i<win_size ; i++)
        debug_decoder << ys->norm_sample(max_index+i) << " ";
      debug_decoder << std::endl << std::endl << std::endl << std::endl;
      #endif

      // check if correlation value exceeds threshold
      if(max_corr > threshold) return max_index + win_size;
      else
      {
        log << "│ Preamble detection fail.." << std::endl;
        std::cout << "\t\t\t\t\tPreamble FAIL!!";
        return -1;
      }
    }

    std::vector<float> tag_decoder_impl::tag_detection(sample_information* ys, int index, int n_expected_bit)
    // This method decodes n_expected_bit of data by using previous methods, and returns the vector of the decoded data.
    // index: start point of "data bit", do not decrease half bit!
    {
      std::vector<float> decoded_bits;

      int mask_level = determine_first_mask_level(ys, index);
      int shift = 0;

      float threshold = 0.8f;

      for(int i=0 ; i<n_expected_bit ; i++)
      {
        int idx = index + i*n_samples_TAG_BIT + shift;  // start point of decoding bit with shifting
        int curr_shift = 0;

        int max_bit = decode_single_bit(ys, idx, mask_level);
        float max_corr = ys->corr();

        if(max_corr < threshold)
        {
          for(int j=-SHIFT_SIZE ; j<SHIFT_SIZE ; j++)
          {
            int bit = decode_single_bit(ys, idx+j, mask_level);
            float corr = ys->corr();

            if(corr > max_corr)
            {
              max_corr = corr;
              max_bit = bit;
              curr_shift = j;
            }
          }
        }
        shift += curr_shift;

        #ifdef DEBUG_MESSAGE_DECODER
        debug_decoder << "\t\t\t\t\t**" << i+1 << "th bit **" << std::endl;
        debug_decoder << "threshold= " << threshold << "\tcorr= " << max_corr << "\tdecoded bit= " << max_bit << "\tcurr shift= " << curr_shift << "\tshift= " << shift;
        if(mask_level) debug_decoder << "\thigh start" << std::endl;
        else debug_decoder << "\tlow start" << std::endl;
        for(int j=-curr_shift ; j<n_samples_TAG_BIT-curr_shift ; j++)
          debug_decoder << ys->norm_sample(idx+j) << " ";
        debug_decoder << std::endl << std::endl << std::endl;
        #endif

        if(max_bit) mask_level *= -1; // change mask_level(start level of the next bit) when the decoded bit is 1

        decoded_bits.push_back(max_bit);
      }

      return decoded_bits;
    }

    int tag_decoder_impl::determine_first_mask_level(sample_information* ys, int index)
    // This method searches whether the first bit starts with low level or high level.
    // If the first bit starts with low level, it returns -1.
    // If the first bit starts with high level, it returns 1.
    // index: start point of "data bit", do not decrease half bit!
    {
      decode_single_bit(ys, index, -1);
      float low_level_corr = ys->corr();

      decode_single_bit(ys, index, 1);
      if(low_level_corr > ys->corr()) return -1;
      else return 1;
    }

    int tag_decoder_impl::decode_single_bit(sample_information* ys, int index, int mask_level)
    // This method decodes single bit and returns the decoded value and the correlation score.
    // index: start point of "data bit", do not decrease half bit!
    // mask_level: start level of "decoding bit". (-1)low level start, (1)high level start.
    {
      const float masks[2][2][4] = { // first, last elements are extra bits. second, third elements are real signal.
        {{1, -1, 1, -1}, {1, -1, -1, 1}}, // low level start
        {{-1, 1, -1, 1}, {-1, 1, 1, -1}}  // high level start
      };

      if(mask_level == -1) mask_level = 0;  // convert for indexing

      float max_corr = 0.0f;
      int max_index = -1;

      float average_amp = 0.0f;
      for(int j=-(n_samples_TAG_BIT*0.5) ; j<(n_samples_TAG_BIT*1.5) ; j++)
        average_amp += ys->norm_sample(index+j);
      average_amp /= (int)(2*n_samples_TAG_BIT);

      float average_abs_amp = 0.0f;
      for(int j=-(n_samples_TAG_BIT*0.5) ; j<(n_samples_TAG_BIT*1.5) ; j++)
          average_abs_amp += abs(ys->norm_sample(index+j) - average_amp);
      average_abs_amp /= (int)(2*n_samples_TAG_BIT);

      // comepare with two masks (0 or 1)
      for(int i=0 ; i<2 ; i++)
      {
        float corr = 0.0f;
        for(int j=0; j<4; j++)
        {
          int start = index + (j-1)*(n_samples_TAG_BIT*0.5);
          for(int k=0 ; k<(n_samples_TAG_BIT*0.5) ; k++)
          {
            float scaled_amp = (ys->norm_sample(start+k) - average_amp) / average_abs_amp;
            corr += masks[mask_level][i][j] * scaled_amp;
          }
        }

        if(corr > max_corr)
        {
          max_corr = corr;
          max_index = i;
        }
      }

      ys->set_corr(max_corr / (n_samples_TAG_BIT*2));
      return max_index;
    }
  } /* namespace rfid */
} /* namespace gr */
