#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "tag_decoder_impl.h"

namespace gr
{
  namespace rfid
  {
    tag_decoder_impl::sample_information::sample_information()
    {
      _in = NULL;
      _total_size = 0;
      _norm_in.clear();
      _sample.clear();
      _norm_sample.clear();
      _size = 0;

      _center.clear();
      _cluster.clear();
    }

    tag_decoder_impl::sample_information::~sample_information()
    {

    }

    void tag_decoder_impl::sample_information::set_in(gr_complex* __in)
    {
      _in = __in;
    }

    void tag_decoder_impl::sample_information::set_total_size(int __total_size)
    {
      _total_size = __total_size;
    }

    void tag_decoder_impl::sample_information::calc_norm_in(void)
    {
      for(int i=0 ; i<_total_size ; i++)
        _norm_in.push_back(std::sqrt(std::norm(_in[i])));
    }

    void tag_decoder_impl::sample_information::cut_noise_sample(int data_len, int n_samples_TAG_BIT)
    {
      int idx = 1;
      const float threshold = 0.002;
      float average = _norm_in[0];

      for(; idx<_total_size ; idx++)
      {
        average += _norm_in[idx];
        average /= 2;
        if(std::abs(_norm_in[idx] - average) > threshold) break;
      }

      int start = idx;  // start idx of the data sample

      idx += data_len*n_samples_TAG_BIT;
      average = _norm_in[idx];
      int count = 0;

      for(int i=1 ; idx+i<_total_size ; i++)
      {
        average += _norm_in[idx+i];
        average /= 2;

        if(std::abs(_norm_in[idx+i] - average) > threshold)
        {
          count = 0;
          idx += i;
          i = 0;
          average = _norm_in[idx];
        }
        else count++;
        if(count >= 1.5 * n_samples_TAG_BIT) break;
      }

      int end = idx;  // end idx of the data sample

      for(int i=start ; i<=end ; i++)
      {
        _sample.push_back(_in[i]);
        _norm_sample.push_back(_norm_in[i]);
      }

      _size = end - start + 1;
    }

    void tag_decoder_impl::sample_information::push_back_decision(double __decision)
    {
      _decision.push_back(__decision);
    }

    void tag_decoder_impl::sample_information::push_back_center(int __center)
    {
      _center.push_back(__center);
    }

    void tag_decoder_impl::sample_information::erase_center(int index)
    {
      _center.erase(_center.begin() + index);
    }

    void tag_decoder_impl::sample_information::push_back_cluster(int __cluster)
    {
      _cluster.push_back(__cluster);
    }

    void tag_decoder_impl::sample_information::set_cluster(int index, int __cluster)
    {
      _cluster[index] = __cluster;
    }

    void tag_decoder_impl::sample_information::decrease_cluster(int index)
    {
      _cluster[index]--;
    }

    void tag_decoder_impl::sample_information::clear_cluster(void)
    {
      _cluster.clear();
    }

    gr_complex tag_decoder_impl::sample_information::in(int index)
    {
      return _in[index];
    }

    float tag_decoder_impl::sample_information::norm_in(int index)
    {
      return _norm_in[index];
    }

    gr_complex tag_decoder_impl::sample_information::sample(int index)
    {
      return _sample[index];
    }

    float tag_decoder_impl::sample_information::norm_sample(int index)
    {
      return _norm_sample[index];
    }

    int tag_decoder_impl::sample_information::size(void)
    {
      return _size;
    }

    double tag_decoder_impl::sample_information::decision(int index)
    {
      return _decision[index];
    }

    int tag_decoder_impl::sample_information::center(int index)
    {
      return _center[index];
    }

    int tag_decoder_impl::sample_information::center_size(void)
    {
      return _center.size();
    }

    int tag_decoder_impl::sample_information::cluster(int index)
    {
      return _cluster[index];
    }
  } /* namespace rfid */
} /* namespace gr */
