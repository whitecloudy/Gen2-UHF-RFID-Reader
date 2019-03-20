/* -*- c++ -*- */
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

      _decision.clear();
      _center.clear();
      _cluster.clear();
      _n_tag = 0;

      _flip.clear();
      _OFG_node.clear();
      _binary_sample.clear();
    }

    tag_decoder_impl::sample_information::sample_information(gr_complex* __in, int __total_size, int n_samples_TAG_BIT, int mode)
    // mode: 0:RN16, 1:EPC
    {
      _in = __in;
      _total_size = __total_size;
      _norm_in.clear();
      for(int i=0 ; i<_total_size ; i++)
        _norm_in.push_back(std::sqrt(std::norm(_in[i])));

      _sample.clear();
      _norm_sample.clear();
      _size = 0;
      if(mode == 0) cut_noise_sample(TAG_PREAMBLE_BITS + RN16_BITS - 1, n_samples_TAG_BIT);
      else if(mode == 1) cut_noise_sample(TAG_PREAMBLE_BITS + EPC_BITS - 1, n_samples_TAG_BIT);

      _decision.clear();
      _center.clear();
      _cluster.clear();
      _n_tag = 0;

      _flip.clear();
      _OFG_node.clear();
      _binary_sample.clear();
    }

    tag_decoder_impl::sample_information::~sample_information(){}

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

    void tag_decoder_impl::sample_information::set_n_tag(int __n_tag)
    {
      _n_tag = __n_tag;
    }

    void tag_decoder_impl::sample_information::increase_n_tag(void)
    {
      _n_tag++;
    }

    void tag_decoder_impl::sample_information::initialize_flip(void)
    {
      std::vector<int> vec(_center.size());
      for(int i=0 ; i<_center.size() ; i++)
      {
        _flip.push_back(vec);
        for(int j=0 ; j<_center.size() ; j++)
          _flip[i][j] = 0;
      }
    }

    void tag_decoder_impl::sample_information::increase_flip(int index1, int index2)
    {
      _flip[index1][index2]++;
      _flip[index2][index1]++;
    }

    void tag_decoder_impl::sample_information::initialize_OFG(void)
    {
      _OFG_node.resize(_center.size());
      for(int i=0 ; i<_center.size() ; i++)
        _OFG_node[i].state.resize(_n_tag);
    }

    bool tag_decoder_impl::sample_information::is_exist_OFG_link(int base, int target)
    {
      for(int i=0 ; i<_OFG_node[base].link.size() ; i++)
      {
        if(target == _OFG_node[base].link[i]) return true;
      }
      return false;
    }

    void tag_decoder_impl::sample_information::push_back_OFG_link(int index1, int index2)
    {
      _OFG_node[index1].link.push_back(index2);
      _OFG_node[index2].link.push_back(index1);
    }

    bool tag_decoder_impl::sample_information::check_odd_cycle_OFG(int base, int target)
    {
      for(int i=0 ; i<_OFG_node[base].link.size() ; i++)
      {
        for(int j=0 ; j<_OFG_node[target].link.size() ; j++)
        {
          if(_OFG_node[base].link[i] == _OFG_node[target].link[j]) return true;
        }
      }
      return false;
    }

    void tag_decoder_impl::sample_information::set_OFG_layer(int index, int __layer)
    {
      _OFG_node[index].layer = __layer;
    }

    void tag_decoder_impl::sample_information::set_OFG_state(int index1, int index2, int __state)
    {
      _OFG_node[index1].state[index2] = __state;
    }

    void tag_decoder_impl::sample_information::set_binary_sample(void)
    {
      for(int i=0 ; i<_n_tag ; i++)
      {
        std::vector<int> vec;
        for(int j=0 ; j<_size ; j++)
          vec.push_back(_OFG_node[_cluster[j]].state[i]);
        _binary_sample.push_back(vec);
      }
    }

    gr_complex tag_decoder_impl::sample_information::in(int index)
    {
      return _in[index];
    }

    int tag_decoder_impl::sample_information::total_size(void)
    {
      return _total_size;
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

    int tag_decoder_impl::sample_information::cluster_size(int index)
    {
      int count = 0;
      for(int i=0 ; i<_size ; i++)
        if(_cluster[i] == index) count++;
      return count;
    }

    int tag_decoder_impl::sample_information::n_tag(void)
    {
      return _n_tag;
    }

    int tag_decoder_impl::sample_information::flip(int index1, int index2)
    {
      return _flip[index1][index2];
    }

    int tag_decoder_impl::sample_information::OFG_layer(int index)
    {
      return _OFG_node[index].layer;
    }

    int tag_decoder_impl::sample_information::OFG_state(int index1, int index2)
    {
      return _OFG_node[index1].state[index2];
    }

    int tag_decoder_impl::sample_information::OFG_link(int index1, int index2)
    {
      return _OFG_node[index1].link[index2];
    }

    int tag_decoder_impl::sample_information::OFG_link_size(int index)
    {
      return _OFG_node[index].link.size();
    }

    int tag_decoder_impl::sample_information::binary_sample(int index1, int index2)
    {
      return _binary_sample[index1][index2];
    }
  } /* namespace rfid */
} /* namespace gr */
