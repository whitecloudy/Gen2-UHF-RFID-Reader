/* -*- c++ -*- */
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
      ys->initialize_flip();

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
      debug_OFG << std::endl << std::endl;
      #endif
    }

    bool tag_decoder_impl::construct_OFG(sample_information* ys)
    {
      struct Compound
      {
        int id;
        float value;
      };

      std::vector<std::vector<Compound>> p_trans(ys->center_size(), std::vector<Compound>(ys->center_size()));
      for(int i=0 ; i<ys->center_size() ; i++)
      {
        for(int j=0 ; j<ys->center_size() ; j++)
        {
          p_trans[i][j].id = j;
          p_trans[i][j].value = ys->flip(i, j);
        }
      }

      // sort by decreasing order for each row
      for(int i=0 ; i<ys->center_size() ; i++)
      {
        // move self flip to the right
        Compound temp = p_trans[i][i];
        p_trans[i][i] = p_trans[i][ys->center_size()-1];
        p_trans[i][ys->center_size()-1] = temp;

        // selection sort
        for(int j=0 ; j<ys->center_size()-2 ; j++)
        {
          for(int k=j+1 ; k<ys->center_size()-1 ; k++)
          {
            if(p_trans[i][j].value < p_trans[i][k].value)
            {
              Compound temp = p_trans[i][j];
              p_trans[i][j] = p_trans[i][k];
              p_trans[i][k] = temp;
            }
          }
        }
      }

      #ifdef DEBUG_MESSAGE_OFG
      debug_OFG << "\t\t\t\t\t** p_trans **" << std::endl;
      for(int i=0 ; i<ys->center_size() ; i++)
      {
        debug_OFG << i << ":\t";
        for(int j=0 ; j<ys->center_size() ; j++)
          debug_OFG << p_trans[i][j].id << ":" << p_trans[i][j].value << "\t";
        debug_OFG << std::endl;
      }
      debug_OFG << std::endl << std::endl;
      #endif

      for(int i=0 ; i<ys->center_size() ; i++)
      {
        // set last column as sum of all flip
        for(int j=0 ; j<ys->center_size()-1 ; j++)
          p_trans[i][ys->center_size()-1].value += p_trans[i][j].value;

        // calculate ratio
        for(int j=0 ; j<ys->center_size()-1 ; j++)
          p_trans[i][j].value /= p_trans[i][ys->center_size()-1].value;
      }

      std::vector<Compound> conf(ys->center_size());

      for(int i=0 ; i<ys->center_size() ; i++)
      {
        conf[i].id = i;
        if(p_trans[i][ys->n_tag()].value == 0) conf[i].value = 0;
        else conf[i].value = p_trans[i][ys->n_tag()-1].value / p_trans[i][ys->n_tag()].value;
      }

      // sort by decreasing order for each row
      for(int i=0 ; i<ys->center_size()-1 ; i++)
      {
        for(int j=i+1 ; j<ys->center_size() ; j++)
        {
          if(conf[i].value < conf[j].value)
          {
            Compound temp = conf[i];
            conf[i] = conf[j];
            conf[j] = temp;
          }
        }
      }

      #ifdef DEBUG_MESSAGE_OFG
      debug_OFG << "\t\t\t\t\t** conf **" << std::endl;
      for(int i=0 ; i<ys->center_size() ; i++)
      {
        debug_OFG << conf[i].id << ":" << conf[i].value << "\t";
      }
      debug_OFG << std::endl << std::endl << std::endl;
      #endif

      ys->initialize_OFG();
      for(int i=0 ; i<ys->center_size() ; i++)
      {
        int base = conf[i].id;

        for(int j=0 ; ys->OFG_link_size(base)<ys->n_tag() ; j++)
        {
          if(j == ys->center_size() - 1)  // construct error
          {
            #ifdef DEBUG_MESSAGE_OFG
            debug_OFG << "\t\t\t\t\t** OFG_link error **";
            #endif
            return false;
          }

          int target = p_trans[base][j].id;
          if(ys->is_exist_OFG_link(base, target)) continue;
          if(ys->OFG_link_size(target) >= ys->n_tag()) continue;
          if(ys->check_odd_cycle_OFG(base, target)) continue;
          ys->push_back_OFG_link(base, target);
        }
      }

      #ifdef DEBUG_MESSAGE_OFG
      debug_OFG << "\t\t\t\t\t** OFG_link **" << std::endl;
      for(int i=0 ; i<ys->center_size() ; i++)
      {
        debug_OFG << i << ":\t";
        for(int j=0 ; j<ys->OFG_link_size(i) ; j++)
          debug_OFG << ys->OFG_link(i, j) << " ";
        debug_OFG << std::endl;
      }
      debug_OFG << std::endl << std::endl;
      #endif

      return true;
    }

    void tag_decoder_impl::determine_OFG_state(sample_information* ys)
    {
      ys->set_OFG_layer(0, 0);

      for(int i=0 ; i<ys->n_tag() ; i++)
      {
        ys->set_OFG_layer(ys->OFG_link(0, i), 1);
        ys->set_OFG_state(ys->OFG_link(0, i), i, 1);
      }

      for(int i=2 ; i<=ys->n_tag() ; i++) // layer
      {
        for(int base=0 ; base<ys->center_size() ; base++)  // for all nodes
        {
          if(ys->OFG_layer(base) == i-1) // if base node is parent node
          {
            for(int j=0 ; j<ys->n_tag() ; j++)
            {
              // set base node's jth link as target
              int target = ys->OFG_link(base, j);

              // continue when target node's layer is less than base node
              if(ys->OFG_layer(target) < i) continue;
              // set target node's layer
              ys->set_OFG_layer(target, i);

              // set target node's state
              for(int k=0 ; k<ys->n_tag() ; k++)
                if(ys->OFG_state(base, k) == 1) ys->set_OFG_state(target, k, 1);
            }
          }
        }
      }

      #ifdef DEBUG_MESSAGE_OFG
      debug_OFG << "\t\t\t\t\t** OFG_state **" << std::endl;
      for(int i=0 ; i<ys->center_size() ; i++)
      {
        debug_OFG << i << ":\t";
        for(int j=0 ; j<ys->n_tag() ; j++)
          debug_OFG << ys->OFG_state(i, j);
        debug_OFG << std::endl;
      }
      debug_OFG << std::endl << std::endl << std::endl;
      #endif
    }
/*
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
    }*/
  } /* namespace rfid */
} /* namespace gr */
