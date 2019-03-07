#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

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

    void tag_decoder_impl::center_identification()
    {
      std::vector<int> local_density;
      std::vector<double> local_distance;
      std::vector<double> continuity;
      std::vector<double> decision;

      const int half_time_window = n_samples_TAG_BIT / 2;

      double max_local_distance = 0;
      double min_local_distance = 1;
      double max_decision = 0;

      // calculate local density and continuity
      for(int i=0 ; i<size ; i++)
      {
        int current_local_density = -1;
        double current_continuity = -1;
        int count = 0;

        for(int j=0 ; j<size ; j++)
        {
          if(IQ_distance(sample[i], sample[j]) < CUTOFF_DISTANCE)
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

      // calculate local distance
      for(int i=0 ; i<size ; i++)
      {
        double min_distance = 1;

        for(int j=0 ; j<size ; j++)
        {
          if(local_density[i] >= local_density[j]) continue;

          double distance = IQ_distance(sample[i], sample[j]);
          if(distance < min_distance) min_distance = distance;
        }

        if(min_distance != 1)
        {
          if(min_distance > max_local_distance) max_local_distance = min_distance;
          if(min_distance < min_local_distance) min_local_distance = min_distance;
        }
        local_distance.push_back(min_distance);
      }

      // normalize local distance
      for(int i=0 ; i<size ; i++)
      {
        if(local_distance[i] == 1) continue;  // the maximum center
        local_distance[i] = 0.8 * ((local_distance[i] - min_local_distance) / (max_local_distance - min_local_distance));
      }

      // calculate decision
      for(int i=0 ; i<size ; i++)
      {
        double current_decision = local_density[i] * local_distance[i] * continuity[i];
        if(current_decision > max_decision) max_decision = current_decision;
        decision.push_back(current_decision);
      }

      // center identification
      for(int i=0 ; i<size ; i++)
      {
        if(decision[i] > max_decision * 0.1) center.push_back(i);
      }

      // remove center which the location is same
      for(int i=0 ; i<center.size() ; i++)
      {
        for(int j=i ; j<center.size() ; j++)
        {
          if(i == j) continue;

          if(sample[center[i]] == sample[center[j]])
          {
            center.erase(center.begin() + j);
            j--;
          }
        }
      }
    }

    float tag_decoder_impl::norm_2dim_gaussian_pdf(const gr_complex value, const gr_complex mean, const float standard_deviation)
    {
      return std::exp(- (std::pow(value.real() - mean.real(), 2) + std::pow(value.imag() - mean.imag(), 2)) / (2 * std::pow(standard_deviation, 2)));
    }

    float tag_decoder_impl::pd_i_k(const int i, const int k)
    {
      return norm_2dim_gaussian_pdf(sample[i], sample[center[k]], CUTOFF_DISTANCE);
    }

    float tag_decoder_impl::pt_i_k(const int i, const int k)
    {
      int count = 0;
      int window_size = 0;

      int half_time_window = n_samples_TAG_BIT / 2;
      int start = ((i - half_time_window) >= 0) ? (i - half_time_window) : 0;
      int end = ((i + half_time_window) <= size) ? (i + half_time_window) : size;

      for(int j=start ; j<end ; j++)
      {
        if(clustered_id[j] == -1) continue; // no count unclustered sample

        window_size++;
        if(clustered_id[j] == k) count++;
      }

      if(window_size == 0) return 0;
      return (float)count / window_size;
    }

    int tag_decoder_impl::max_id_pcluster_i(const int i)
    {
      float max_pcluster = 0;
      int max_id;

      for(int k=0 ; k<center.size() ; k++)
      {
        float pcluster = pd_i_k(i, k) * pt_i_k(i, k);
        if(pcluster > max_pcluster)
        {
          max_pcluster = pcluster;
          max_id = k;
        }
      }

      return max_id;
    }

    float tag_decoder_impl::max_value_pcluster_i(const int i)
    {
      float max_pcluster = 0;

      for(int k=0 ; k<center.size() ; k++)
      {
        float pcluster = pd_i_k(i, k) * pt_i_k(i, k);
        if(pcluster > max_pcluster) max_pcluster = pcluster;
      }

      return max_pcluster;
    }

    void tag_decoder_impl::sample_clustering()
    {
      float threshold = 0.4;

      for(int i=0 ; i<size ; i++)
      {
        int candidate_id = -1;
        bool chk = true;

        for(int k=0 ; k<center.size() ; k++)
        {
          if(pd_i_k(i, k) > threshold)
          {
            if(candidate_id == -1) candidate_id = k;  // first candidate
            else  // when the candidate is more than one, need to calculate pcluster
            {
              chk = false;
              break;
            }
          }
        }

        if(chk) clustered_id.push_back(candidate_id);
        else clustered_id.push_back(-1);
      }

      for(int i=0 ; i<size ; i++)
      {
        if(clustered_id[i] != -1) continue;  // already clustered
        clustered_id[i] = max_id_pcluster_i(i);
      }
    }

    void tag_decoder_impl::print_cluster_sample(const std::string filename)
    {
      std::ofstream file(filename, std::ios::app);

      file << "\t\t\t\t\t** center_id **" << std::endl;
      for(int i=0 ; i<center.size() ; i++)
        file << center[i] << " ";
      file << std::endl << std::endl;

      file << "\t\t\t\t\t** center(I) **" << std::endl;
      for(int i=0 ; i<center.size() ; i++)
        file << sample[center[i]].real() << " ";
      file << std::endl;
      file << "\t\t\t\t\t** center(Q) **" << std::endl;
      for(int i=0 ; i<center.size() ; i++)
        file << sample[center[i]].imag() << " ";
      file << std::endl << std::endl;

      for(int i=0 ; i<center.size() ; i++)
      {
        file << "\t\t\t\t\t** cluster " << i << " (I) **" << std::endl;
        for(int j=0 ; j<size ; j++)
        {
          if(clustered_id[j] == i) file << sample[j].real() << " ";
        }
        file << std::endl;
        file << "\t\t\t\t\t** cluster " << i << " (Q) **" << std::endl;
        for(int j=0 ; j<size ; j++)
        {
          if(clustered_id[j] == i) file << sample[j].imag() << " ";
        }
        file << std::endl << std::endl;
      }

      file.close();
    }
  }
}
