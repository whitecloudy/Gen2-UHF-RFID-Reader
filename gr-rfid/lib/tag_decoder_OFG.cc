#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "tag_decoder_impl.h"

namespace gr
{
  namespace rfid
  {
    void tag_decoder_impl::count_flip(sample_information* ys)
    {
      ys->allocate_flip();
      for(int i=0 ; i<ys->center_size() ; i++)
      {
        ys->allocate_flip(i);
        for(int j=0 ; j<ys->center_size() ; j++)
          ys->set_flip(i, j, 0);
      }

      for(int i=0 ; i<ys->size()-1 ; i++)
      {
        if(ys->cluster(i) == ys->cluster(i+1)) continue;
        ys->increase_flip(ys->cluster(i), ys->cluster(i+1));
      }

      #ifdef DEBUG_MESSAGE_OFG
      debug_OFG << "\t\t\t\t\t** flip **" << std::endl;
      for(int i=0 ; i<ys->center_size() ; i++)
      {
        for(int j=0 ; j<ys->center_size() ; j++)
          debug_OFG << ys->flip(i, j) << " ";
        debug_OFG << std::endl;
      }
      debug_OFG << std::endl << std::endl << std::endl << std::endl;
      #endif
    }

    int tag_decoder_impl::check_odd_cycle_OFG(OFG_node* OFG, int start, int compare, int check, std::vector<int> stack)
    {
      if(start == compare)
      {
        if(check == 1) return 1;
        else return -1;
      }

      for(int i=0 ; i<stack.size() ; i++)
      {
        if(stack[i] == compare) return 0;
      }

      check *= -1;
      stack.push_back(compare);

      for(int i=0 ; i<OFG[compare].link.size() ; i++)
      {
        if(check_odd_cycle_OFG(OFG, start, OFG[compare].link[i], check, stack) == -1) return -1;
      }

      return 0;
    }

    void tag_decoder_impl::construct_OFG(OFG_node* OFG, int** flip_info, int size, int n_tag)
    {
      std::ofstream flipf("flip", std::ios::app);

      int** flip = new int*[size];
      for(int i=0 ; i<size ; i++)
      {
        flip[i] = new int[size];

        for(int j=0 ; j<size ; j++)
          flip[i][j] = flip_info[i][j] + flip_info[j][i];
      }

      for(int i=0 ; i<size ; i++)
      {
        for(int j=0 ; j<size ; j++)
          flipf << flip[i][j] << " ";
        flipf << std::endl;
      }
      flipf << std::endl;

      float* conf = new float[size];

      int** link_id = new int*[size];
      for(int i=0 ; i<size ; i++)
      {
        link_id[i] = new int[size];

        for(int j=0 ; j<size ; j++)
        {
          link_id[i][j] = j;
        }

        for(int j=0 ; j<size-1 ; j++)
        {
          for(int k=j+1 ; k<size ; k++)
          {
            if(flip[i][j] < flip[i][k])
            {
              int temp = flip[i][j];
              flip[i][j] = flip[i][k];
              flip[i][k] = temp;

              temp = link_id[i][j];
              link_id[i][j] = link_id[i][k];
              link_id[i][k] = temp;
            }
          }
        }

        if(flip[i][n_tag] == 0) conf[i] = 1;
        else conf[i] = flip[i][n_tag-1] / flip[i][n_tag];
      }

      for(int i=0 ; i<size ; i++)
      {
        for(int j=0 ; j<size ; j++)
          flipf << flip[i][j] << " ";
        flipf << std::endl;
      }
      flipf << std::endl;

      for(int i=0 ; i<size ; i++)
      {
        for(int j=0 ; j<size ; j++)
          flipf << link_id[i][j] << " ";
        flipf << std::endl;
      }
      flipf << std::endl;

      int* conf_id = new int[size];
      for(int i=0 ; i<size ; i++)
        conf_id[i] = i;

      for(int i=0 ; i<size-1 ; i++)
      {
        for(int j=i+1 ; j<size ; j++)
        {
          if(conf[i] < conf[j])
          {
            float temp = conf[i];
            conf[i] = conf[j];
            conf[j] = temp;

            int temp_id = conf_id[i];
            conf_id[i] = conf_id[j];
            conf_id[j] = temp_id;
          }
        }
      }

      for(int i=0 ; i<size ; i++)
      {
        for(int j=0 ; OFG[conf_id[i]].link.size()<n_tag ; j++)
        {
          int candidate_id = link_id[conf_id[i]][j];
          int k;

          for(k=0 ; k<OFG[conf_id[i]].link.size() ; k++)
          {
            if(candidate_id == OFG[conf_id[i]].link[k]) break;
          }

          if(k == OFG[conf_id[i]].link.size())
          {
            OFG[conf_id[i]].link.push_back(candidate_id);
            for(int x=0 ; x<OFG[conf_id[i]].link.size() ; x++)
            {
              std::vector<int> stack;
              if(check_odd_cycle_OFG(OFG, conf_id[i], OFG[conf_id[i]].link[x], -1, stack) == -1)
                OFG[conf_id[i]].link.pop_back();
            }
          }
        }
      }

      flipf<<std::endl<<std::endl;
      for(int i=0 ; i<size ; i++)
      {
        for(int j=0 ; j<n_tag ; j++)
          flipf << OFG[i].link[j] << " ";
        flipf << std::endl;
      }

      flipf.close();

      for(int i=0 ; i<size ; i++)
      {
        delete flip[i];
        delete link_id[i];
      }

      delete flip;
      delete conf;
      delete link_id;
    }

    void tag_decoder_impl::determine_OFG_state(OFG_node* OFG, int size, int n_tag)
    {
      OFG[0].layer = 0;

      for(int i=0 ; i<n_tag ; i++)
      {
        OFG[OFG[0].link[i]].layer = 1;
        OFG[OFG[0].link[i]].state[i] = 1;
      }

      std::ofstream flipf("flip", std::ios::app);
      for(int i=0 ; i<size ; i++)
      {
        flipf << "i=" << i;
        for(int j=0 ; j<n_tag ; j++)
          flipf << " " << OFG[i].state[j];
        flipf << std::endl;
      }
      flipf<<std::endl<<std::endl;

      for(int i=2 ; i<=n_tag ; i++)
      {
        for(int j=0 ; j<size ; j++)
        {
          if(OFG[j].layer == i-1)
          {
            for(int k=0 ; k<n_tag ; k++)
            {
              if(OFG[OFG[j].link[k]].layer < i) continue;
              OFG[OFG[j].link[k]].layer = i;

              for(int x=0 ; x<n_tag ; x++)
              {
                if(OFG[j].state[x] == 1)
                  OFG[OFG[j].link[k]].state[x] = 1;
              }
            }
          }
        }

        for(int i=0 ; i<size ; i++)
        {
          flipf << "i=" << i;
          for(int j=0 ; j<n_tag ; j++)
            flipf << " " << OFG[i].state[j];
          flipf << std::endl;
        }
        flipf<<std::endl<<std::endl;
      }

      flipf.close();
    }

    void tag_decoder_impl::extract_parallel_samplesss(std::vector<int>* extracted_sample, const std::vector<int> clustered_idx, const OFG_node* OFG, int n_tag)
    {
      for(int i=0 ; i<clustered_idx.size() ; i++)
      {
        for(int j=0 ; j<n_tag ; j++)
          extracted_sample[j].push_back(OFG[clustered_idx[i]].state[j]);
      }

      std::ofstream flipf("flip", std::ios::app);
      for(int i=0 ; i<n_tag ; i++)
      {
        flipf << "\t*** tag " << i << " ***" << std::endl;
        for(int j=0 ; j<clustered_idx.size() ; j++)
          flipf << extracted_sample[i][j] << " ";
        flipf << std::endl << std::endl;
      }
      flipf.close();
    }
  } /* namespace rfid */
} /* namespace gr */
