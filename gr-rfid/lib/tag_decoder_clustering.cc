#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <cmath>
#include "tag_decoder_impl.h"

#define CUTOFF_DISTANCE 0.001

namespace gr
{
  namespace rfid
  {
    double tag_decoder_impl::IQ_distance(const gr_complex p1, const gr_complex p2)
    {
      return std::sqrt(std::pow((p1.real() - p2.real()), 2) + std::pow((p1.imag() - p2.imag()), 2));
    }

    std::vector<int> tag_decoder_impl::clustering_algorithm(const std::vector<gr_complex> in, const int size)
    {
      std::vector<int> center_idx;

      std::vector<int> local_density;
      std::vector<double> local_distance;
      std::vector<double> continuity;
      std::vector<double> decision;

      const int half_time_window = n_samples_TAG_BIT / 2;

      double max_local_distance = 0;
      double min_local_distance = 1;
      double max_decision = 0;
        std::ofstream iq("iq", std::ios::app);
        for(int i=0 ; i<size ; i++)
          iq << in[i].real() << " ";
        iq << std::endl;
        for(int j=0 ; j<size ; j++)
          iq << in[j].imag() << " ";
        iq.close();

      for(int i=0 ; i<size ; i++)
      {
        int current_local_density = -1;
        double current_continuity = -1;
        int count = 0;

        for(int j=0 ; j<size ; j++)
        {
          if(IQ_distance(in[i], in[j]) < CUTOFF_DISTANCE)
          {
            current_local_density++;
            if(j >= i - half_time_window && j <= i + half_time_window)
            {
              current_continuity = current_continuity + 1;
              count++;
            }
          }
        }

        local_density.push_back(current_local_density);
        current_continuity /= count;
        continuity.push_back(current_continuity);
      }
      for(int i=0 ; i<size ; i++)
      {
        double min_distance = 1;

        for(int j=0 ; j<size ; j++)
        {
          if(local_density[i] >= local_density[j]) continue;

          double distance = IQ_distance(in[i], in[j]);
          if(distance < min_distance) min_distance = distance;
        }

        if(min_distance != 1)
        {
          if(min_distance > max_local_distance) max_local_distance = min_distance;
          if(min_distance < min_local_distance) min_local_distance = min_distance;
        }
        local_distance.push_back(min_distance);
      }

      for(int i=0 ; i<size ; i++)
      {
        if(local_distance[i] == 1) continue;
        local_distance[i] = 0.8 * ((local_distance[i] - min_local_distance) / (max_local_distance - min_local_distance));
      }

      for(int i=0 ; i<size ; i++)
      {
        double current_decision = local_density[i] * local_distance[i] * continuity[i];
        if(current_decision > max_decision) max_decision = current_decision;
        decision.push_back(current_decision);
      }

      // debug

        std::ofstream cluster("cluster", std::ios::app);
        cluster << " ** local density **" << std::endl;
        for(int i=0 ; i<size ; i++)
          cluster << local_density[i] << " ";
        cluster << std::endl << "** local distance **" << std::endl;
        for(int i=0 ; i<size; i++)
          cluster << local_distance[i] << " ";
        cluster << std::endl << "** continuity **" << std::endl;
        for(int i=0 ; i<size; i++)
          cluster << continuity[i] << " ";
        cluster << std::endl << "** decision **" << std::endl;
        for(int i=0 ; i<size; i++)
          cluster << decision[i] << " ";
        cluster << std::endl;

      int count = 0;

      for(int i=0 ; i<size ; i++)
      {
        if(decision[i] > max_decision * 0.1)
        {
          center_idx.push_back(i);
          count++;
        }
      }

      cluster << " **center idx**" << std::endl;
      for(int i=0 ; i<center_idx.size() ; i++)
        cluster << center_idx[i] << " ";
      cluster << std::endl << "** center I " << std::endl;
      for(int i=0 ; i<center_idx.size() ; i++)
        cluster << in[center_idx[i]].real() << " ";
      cluster << std::endl << "** center Q " << std::endl;
      for(int i=0 ; i<center_idx.size() ; i++)
        cluster << in[center_idx[i]].imag() << " ";
      cluster << std::endl;

      for(int i=0 ; i<center_idx.size() ; i++)
      {
        for(int j=i ; j<center_idx.size() ; j++)
        {
          if(i == j) continue;

          if(in[center_idx[i]].real() == in[center_idx[j]].real() && in[center_idx[i]].imag() == in[center_idx[j]].imag())
          //if(IQ_distance(in[center_idx[i]], in[center_idx[j]]) < CUTOFF_DISTANCE)
          {
            center_idx.erase(center_idx.begin() + j);
            j--;
          }
        }
      }

      cluster << " **center idx**" << std::endl;
      for(int i=0 ; i<center_idx.size() ; i++)
        cluster << center_idx[i] << " ";
      cluster << std::endl << "** center I " << std::endl;
      for(int i=0 ; i<center_idx.size() ; i++)
        cluster << in[center_idx[i]].real() << " ";
      cluster << std::endl << "** center Q " << std::endl;
      for(int i=0 ; i<center_idx.size() ; i++)
        cluster << in[center_idx[i]].imag() << " ";
      cluster << std::endl;
      cluster.close();

      int n_tag = -1;
      {
        int size = center_idx.size();
        while(size)
        {
          size /= 2;
          n_tag++;
        }
      }
/*
      int erase = center_idx.size() - pow(2, n_tag);
      for(; erase>0 ; erase--)
      {
        double min_distance = 0;
        int min_idx = -1;

        for(int i=0 ; i<center_idx.size() ; i++)
        {
          for(int j=i ; j<center_idx.size() ; j++)
          {
            if(i == j) continue;

            double distance = IQ_distance(in[center_idx[i]], in[center_idx[j]]);
            if(distance < min_distance)
            {
              min_distance = distance;
              min_idx = j;
            }
          }
        }

        center_idx.erase(center_idx.begin() + min_idx);
      }
*/
      if(count == 0) center_idx.push_back(-1);

      return center_idx;
    }

    float tag_decoder_impl::norm_2dim_gaussian_pdf(const gr_complex value, const gr_complex mean, const float standard_deviation)
    {
      return std::exp(- (std::pow(value.real() - mean.real(), 2) + std::pow(value.imag() - mean.imag(), 2)) / (2 * std::pow(standard_deviation, 2)));
    }

    float tag_decoder_impl::pd_i_k(const std::vector<gr_complex> in, const int i, const int k)
    {
      return norm_2dim_gaussian_pdf(in[i], in[k], CUTOFF_DISTANCE);
    }

    float tag_decoder_impl::pt_i_k(const std::vector<int> clustered_idx, const int size, const int i, const int k)
    {
      int count = 0;
      int window_size = 0;

      int half_time_window = n_samples_TAG_BIT / 2;
      int start = ((i - half_time_window) >= 0) ? (i - half_time_window) : 0;
      int end = ((i + half_time_window) <= size) ? (i + half_time_window) : size;

      for(int j=start ; j<end ; j++)
      {
        if(clustered_idx[j] == -1) continue;

        window_size++;
        if(clustered_idx[j] == k) count++;
      }

      if(window_size == 0) return 0;
      return (float)count / window_size;
    }

    int tag_decoder_impl::max_id_pcluster_i(const std::vector<gr_complex> in, const std::vector<int> clustered_idx, const int size, const std::vector<int> center, const int i)
    {
      float max_pcluster = 0;
      int max_id;

      for(int k=0 ; k<center.size() ; k++)
      {
        float pcluster = pd_i_k(in, i, center[k]) * pt_i_k(clustered_idx, size, i, k);
        if(pcluster > max_pcluster)
        {
          max_pcluster = pcluster;
          max_id = k;
        }
      }

      return max_id;
    }

    float tag_decoder_impl::max_value_pcluster_i(const std::vector<gr_complex> in, const std::vector<int> clustered_idx, const int size, const std::vector<int> center, const int i)
    {
      float max_pcluster = 0;

      for(int k=0 ; k<center.size() ; k++)
      {
        float pcluster = pd_i_k(in, i, center[k]) * pt_i_k(clustered_idx, size, i, k);
        if(pcluster > max_pcluster) max_pcluster = pcluster;
      }

      return max_pcluster;
    }

    std::vector<int> tag_decoder_impl::sample_clustering(const std::vector<gr_complex> in, const int size, const std::vector<int> center)
    {
      std::vector<int> clustered_idx;
      float threshold = 0.4;

      for(int i=0 ; i<size ; i++)
      {
        int candidate_id = -1;
        bool chk = true;

        for(int k=0 ; k<center.size() ; k++)
        {
          if(pd_i_k(in, i, center[k]) > threshold)
          {
            if(candidate_id == -1) candidate_id = k;
            else
            {
              chk = false;
              break;
            }
          }
        }

        if(chk) clustered_idx.push_back(candidate_id);
        else clustered_idx.push_back(-1);
      }

      for(int i=0 ; i<size ; i++)
      {
        if(clustered_idx[i] != -1) continue;
        clustered_idx[i] = max_id_pcluster_i(in, clustered_idx, size, center, i);
      }

      std::ofstream cluster("cluster", std::ios::app);
    /*  cluster << "** pd_i_0 **" << std::endl;
      for(int i=0 ; i<size ; i++)
      {
        cluster << pd_i_k(in, i, center[0]) << " ";
      }
      cluster << std::endl;
*/
      cluster << "** clustered ** " << std::endl;
      for(int i=0 ; i<size ; i++)
      {
        cluster << clustered_idx[i] << " ";
      }
      cluster << std::endl;
      cluster.close();

      return clustered_idx;
    }

    void tag_decoder_impl::print_cluster_sample(const std::string filename, const std::vector<gr_complex> in, const std::vector<int> clustered_idx, const int size, const std::vector<int> center)
    {
      std::ofstream file(filename, std::ios::app);

      file << "\t\t\t\t\t** center_id **" << std::endl;
      for(int i=0 ; i<center.size() ; i++)
        file << center[i] << " ";
      file << std::endl << std::endl;

      file << "\t\t\t\t\t** center(I) **" << std::endl;
      for(int i=0 ; i<center.size() ; i++)
        file << in[center[i]].real() << " ";
      file << std::endl;
      file << "\t\t\t\t\t** center(Q) **" << std::endl;
      for(int i=0 ; i<center.size() ; i++)
        file << in[center[i]].imag() << " ";
      file << std::endl << std::endl;

      for(int i=0 ; i<center.size() ; i++)
      {
        file << "\t\t\t\t\t** cluster " << i << " (I) **" << std::endl;
        for(int j=0 ; j<size ; j++)
        {
          if(clustered_idx[j] == i) file << in[j].real() << " ";
        }
        file << std::endl;
        file << "\t\t\t\t\t** cluster " << i << " (Q) **" << std::endl;
        for(int j=0 ; j<size ; j++)
        {
          if(clustered_idx[j] == i) file << in[j].imag() << " ";
        }
        file << std::endl << std::endl;
      }

      file.close();
    }
  }
}
