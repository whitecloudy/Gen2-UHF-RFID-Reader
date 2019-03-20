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
/*    int tag_decoder_impl::tag_sync(float* norm_in, int size)
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
      for(int i=0 ; i<size-win_size ; i++)  // i: start point
      {
        // calculate average_amp (threshold)
        float average_amp = 0.0f;
        for(int j=0 ; j<win_size ; j++)
          average_amp += norm_in[i+j];
        average_amp /= win_size;

        // calculate normalize_factor
        float standard_deviation = 0.0f;
        for(int j=0 ; j<win_size ; j++)
          standard_deviation += pow(norm_in[i+j] - average_amp, 2);
        standard_deviation /= win_size;
        standard_deviation = sqrt(standard_deviation);

        // calculate correlation value
        float corr_candidates[2] = {0.0f};
        for(int j=0 ; j<2*TAG_PREAMBLE_BITS ; j++)  // j: half_bit index of TAG_PREAMBLE
        {
          for(int k=0 ; k<(n_samples_TAG_BIT/2.0) ; k++)
          {
            for(int m=0 ; m<2 ; m++)  // m: index of TAG_PREAMBLE type
                corr_candidates[m] += TAG_PREAMBLE[m][j] * ((norm_in[i + j*(int)(n_samples_TAG_BIT/2.0) + k] - average_amp) / standard_deviation);
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

      #ifdef DEBUG_MESSAGE
      {
        std::ofstream debug((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str(), std::ios::app);
        debug << "threshold= " << threshold << ", corr= " << max_corr << ", index=" << max_index << std::endl;
        debug << "\t\t\t\t\t** preamble samples **" << std::endl;
        for(int i=0 ; i<win_size ; i++)
          debug << norm_in[max_index+i] << " ";
        debug << std::endl << "\t\t\t\t\t** preamble samples **" << std::endl << std::endl << std::endl << std::endl;
        debug.close();
      }
      #endif

      // check if correlation value exceeds threshold
      if(max_corr > threshold) return max_index + win_size;
      else return -1;
    }

    int tag_decoder_impl::determine_first_mask_level(float* norm_in, int index)
    // This method searches whether the first bit starts with low level or high level.
    // If the first bit starts with low level, it returns -1.
    // If the first bit starts with high level, it returns 0.
    // index: start point of "data bit", do not decrease half bit!
    {
      float max_max_corr = 0.0f;
      int max_max_index = -1;

      for(int k=0 ; k<2 ; k++)
      {
        float max_corr = 0.0f;
        int max_index = decode_single_bit(norm_in, index, k, &max_corr);

        if(max_corr > max_max_corr)
        {
          max_max_corr = max_corr;
          max_max_index = k;
        }
      }

      if(max_max_index == 0) max_max_index = -1;
      return max_max_index;
    }

    int tag_decoder_impl::decode_single_bit(float* norm_in, int index, int mask_level, float* ret_corr)
    // This method decodes single bit and returns the decoded value and the correlation score.
    // index: start point of "data bit", do not decrease half bit!
    // mask_level: start level of "decoding bit". (-1)low level start, (1)high level start.
    {

      std::ofstream time("time/time", std::ios::app);

      const float masks[2][2][4] = { // first, last elements are extra bits. second, third elements are real signal.
        {{1, -1, 1, -1}, {1, -1, -1, 1}}, // low level start
        {{-1, 1, -1, 1}, {-1, 1, 1, -1}}  // high level start
      };

      if(mask_level == -1) mask_level = 0;  // convert for indexing

      float max_corr = 0.0f;
      int max_index = -1;

      clock_t start, end;
      start = clock();

      float average_amp = 0.0f;
      for(int j=-(n_samples_TAG_BIT*0.5) ; j<(n_samples_TAG_BIT*1.5) ; j++)
        average_amp += norm_in[index+j];
      average_amp /= (int)(2*n_samples_TAG_BIT);

      float average_abs_amp = 0.0f;
      for (int j=-(n_samples_TAG_BIT*0.5); j<(n_samples_TAG_BIT*1.5); j++)
          average_abs_amp += abs(norm_in[index+j] - average_amp);
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
            float scaled_amp = (norm_in[start+k] - average_amp) / average_abs_amp;
            corr += masks[mask_level][i][j] * scaled_amp;
          }
        }

        if(corr > max_corr)
        {
          max_corr = corr;
          max_index = i;
        }
      }

      max_corr /= n_samples_TAG_BIT*2;
      (*ret_corr) = max_corr;

      end = clock();
      time << ((double)(end-start)/CLOCKS_PER_SEC) << std::endl;
      time.close();

      return max_index;
    }

    std::vector<float> tag_decoder_impl::tag_detection(float* norm_in, int index, int n_expected_bit)
    // This method decodes n_expected_bit of data by using previous methods, and returns the vector of the decoded data.
    // index: start point of "data bit", do not decrease half bit!
    {
      std::vector<float> decoded_bits;

      int mask_level = determine_first_mask_level(norm_in, index);
      int shift = 0;

      float threshold = 0.8f;

      for(int i=0 ; i<n_expected_bit ; i++)
      {
        int idx = index + i*n_samples_TAG_BIT + shift;  // start point of decoding bit with shifting
        float max_corr = 0.0f;
        int max_index;
        int curr_shift = 0;

        max_index = decode_single_bit(norm_in, idx, mask_level, &max_corr);
        if (max_corr < threshold)
        {
          for (int j=0; j<(SHIFT_SIZE*2+1); j++)
          {
            float corr = 0.0f;
            int index = decode_single_bit(norm_in, idx+j-SHIFT_SIZE, mask_level, &corr);

            if (corr > max_corr)
            {
              max_corr = corr;
              max_index = index;
              curr_shift = j - SHIFT_SIZE;
            }
          }
        }
        shift += curr_shift;

        #ifdef DEBUG_MESSAGE
        {
          std::ofstream debug((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str(), std::ios::app);
          debug << "[" << i+1 << "th bit] corr=" << max_corr << ", curr_shift=" << curr_shift << ", shift=" << shift << ", decoded_bit=" << max_index;
          if(mask_level) debug << " (high start)" << std::endl;
          else debug << " (low start)" << std::endl;
          debug << "\t\t\t\t\t** shifted bit samples **" << std::endl;
          for(int j=idx-SHIFT_SIZE ; j<idx+n_samples_TAG_BIT+SHIFT_SIZE ; j++)
            debug << norm_in[i] << " ";
          debug << std::endl << "\t\t\t\t\t** shifted bit samples **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();
        }
        #endif

        if(max_index) mask_level *= -1; // change mask_level(start level of the next bit) when the decoded bit is 1

        decoded_bits.push_back(max_index);

      }

      return decoded_bits;
    }
*/
  } /* namespace rfid */
} /* namespace gr */
