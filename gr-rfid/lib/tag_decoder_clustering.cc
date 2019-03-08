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

    void tag_decoder_impl::center_identification(sample_information* ys)
    {
      std::vector<int> local_density;
      std::vector<double> local_distance;
      std::vector<double> continuity;

      const int half_time_window = n_samples_TAG_BIT / 2;

      double max_local_distance = 0;
      double min_local_distance = 3e38;
      double max_decision = 0;

      // calculate local density and continuity
      for(int i=0 ; i<ys->size() ; i++)
      {
        int current_local_density = -1;
        double current_continuity = -1;
        int count = 0;

        for(int j=0 ; j<ys->size() ; j++)
        {
          if(IQ_distance(ys->sample(i), ys->sample(j)) < CUTOFF_DISTANCE)
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
      for(int i=0 ; i<ys->size() ; i++)
      {
        double min_distance = 1;

        for(int j=0 ; j<ys->size() ; j++)
        {
          if(local_density[i] >= local_density[j]) continue;

          double distance = IQ_distance(ys->sample(i), ys->sample(j));
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
      for(int i=0 ; i<ys->size() ; i++)
      {
        if(local_distance[i] == 1) continue;  // the maximum center
        local_distance[i] = 0.8 * ((local_distance[i] - min_local_distance) / (max_local_distance - min_local_distance));
      }

      // calculate decision
      for(int i=0 ; i<ys->size() ; i++)
      {
        double current_decision = local_density[i] * local_distance[i] * continuity[i];
        if(current_decision > max_decision) max_decision = current_decision;
        ys->push_back_decision(current_decision);
      }

      // center identification
      for(int i=0 ; i<ys->size() ; i++)
      {
        if(ys->decision(i) > max_decision * 0.1) ys->push_back_center(i);
      }

      // remove center which the location is same
      for(int i=0 ; i<ys->center_size() ; i++)
      {
        for(int j=i ; j<ys->center_size() ; j++)
        {
          if(i == j) continue;

          if(ys->sample(ys->center(i)) == ys->sample(ys->center(j)))
          {
            ys->erase_center(j);
            j--;
          }
        }
      }
    }

    float tag_decoder_impl::norm_2dim_gaussian_pdf(const gr_complex value, const gr_complex mean, const float standard_deviation)
    {
      return std::exp(- (std::pow(value.real() - mean.real(), 2) + std::pow(value.imag() - mean.imag(), 2)) / (2 * std::pow(standard_deviation, 2)));
    }

    float tag_decoder_impl::pd_i_k(sample_information* ys, const int i, const int k)
    {
      return norm_2dim_gaussian_pdf(ys->sample(i), ys->sample(ys->center(k)), CUTOFF_DISTANCE);
    }

    float tag_decoder_impl::pt_i_k(sample_information* ys, const int i, const int k)
    {
      int count = 0;
      int window_size = 0;

      int half_time_window = n_samples_TAG_BIT / 2;
      int start = ((i - half_time_window) >= 0) ? (i - half_time_window) : 0;
      int end = ((i + half_time_window) <= ys->size()) ? (i + half_time_window) : ys->size();

      for(int j=start ; j<end ; j++)
      {
        if(ys->cluster(j) == -1) continue; // no count unclustered sample

        window_size++;
        if(ys->cluster(j) == k) count++;
      }

      if(window_size == 0) return 0;
      return (float)count / window_size;
    }

    int tag_decoder_impl::max_id_pcluster_i(sample_information* ys, const int i)
    {
      float max_pcluster = 0;
      int max_id;

      for(int k=0 ; k<ys->center_size() ; k++)
      {
        float pcluster = pd_i_k(ys, i, k) * pt_i_k(ys, i, k);
        if(pcluster > max_pcluster)
        {
          max_pcluster = pcluster;
          max_id = k;
        }
      }

      return max_id;
    }

    float tag_decoder_impl::max_value_pcluster_i(sample_information* ys, const int i)
    {
      float max_pcluster = 0;

      for(int k=0 ; k<ys->center_size() ; k++)
      {
        float pcluster = pd_i_k(ys, i, k) * pt_i_k(ys, i, k);
        if(pcluster > max_pcluster) max_pcluster = pcluster;
      }

      return max_pcluster;
    }

    void tag_decoder_impl::sample_clustering(sample_information* ys)
    {
      int idx = 0;
      float threshold = 0.4;

      for(int i=0 ; i<ys->size() ; i++)
      {
        if(i == ys->center(idx))
        {
          ys->push_back_cluster(idx);
          idx++;
          continue;
        }

        int candidate_id = -1;
        bool chk = true;

        for(int k=0 ; k<ys->center_size() ; k++)
        {
          if(pd_i_k(ys, i, k) > threshold)
          {
            if(candidate_id == -1) candidate_id = k;  // first candidate
            else  // when the candidate is more than one, need to calculate pcluster
            {
              chk = false;
              break;
            }
          }
        }

        if(chk) ys->push_back_cluster(candidate_id);
        else ys->push_back_cluster(-1);
      }

      for(int i=0 ; i<ys->size() ; i++)
      {
        if(ys->cluster(i) != -1) continue;  // already clustered
        ys->set_cluster(i, max_id_pcluster_i(ys, i));
      }
    }

    void tag_decoder_impl::sample_clustering_after_splitting(sample_information* ys, const int prev_center, const int new_center)
    {
      int idx = 0;
      float threshold = 0.4;

      for(int i=0 ; i<ys->size() ; i++)
      {
        if(ys->cluster(i) == prev_center)
        {
          if(i == ys->center(prev_center)) continue;
          if(i == ys->center(new_center))
          {
            ys->set_cluster(i, new_center);
            continue;
          }

          float pd_prev = pd_i_k(ys, i, prev_center);
          float pd_new = pd_i_k(ys, i, new_center);

          if(pd_prev > threshold && pd_new > threshold) ys->set_cluster(i, -1);
          else if(pd_prev > threshold) ys->set_cluster(i, prev_center);
          else if(pd_new > threshold) ys->set_cluster(i, new_center);
          else ys->set_cluster(i, -1);
        }
      }

      for(int i=0 ; i<ys->size() ; i++)
      {
        if(ys->cluster(i) != -1) continue;  // already clustered

        float pcluster_prev = pd_i_k(ys, i, prev_center) * pt_i_k(ys, i, new_center);
        float pcluster_new = pd_i_k(ys, i, prev_center) * pt_i_k(ys, i, new_center);

        if(pcluster_prev >= pcluster_new) ys->set_cluster(i, prev_center);
        else ys->set_cluster(i, new_center);
      }
    }

    bool tag_decoder_impl::is_power_of_2(sample_information* ys)
    {
      int size = ys->center_size();

      while(size > 1)
      {
        if(size % 2 == 1) return false;
        size /= 2;
      }

      return true;
    }

    void tag_decoder_impl::calc_r_area(sample_information* ys, std::vector<float>* r_area)
    {
      float average_area = 0;
      for(int k=0 ; k<ys->center_size() ; k++)
      {
        int count = 0;
        float average_real = 0;
        float average_imag = 0;
        for(int i=0 ; i<ys->size() ; i++)
        {
          if(ys->cluster(i) == k)
          {
            count++;
            average_real += ys->sample(i).real();
            average_imag += ys->sample(i).imag();
          }
        }
        average_real /= count;
        average_imag /= count;

        float variance_real = 0;
        float variance_imag = 0;
        for(int i=0 ; i<ys->size() ; i++)
        {
          if(ys->cluster(i) == k)
          {
            variance_real += std::pow(ys->sample(i).real(), 2);
            variance_imag += std::pow(ys->sample(i).imag(), 2);
          }
        }
        variance_real /= count;
        variance_imag /= count;

        variance_real -= std::pow(average_real, 2);
        variance_imag -= std::pow(average_imag, 2);

        float area = variance_real * variance_imag;
        (*r_area).push_back(area);
        average_area += area;
      }
      average_area /= ys->center_size();

      for(int k=0 ; k<ys->center_size() ; k++)
      {
        (*r_area)[k] = (*r_area)[k] - average_area;
      }
    }

    void tag_decoder_impl::calc_r_num(sample_information* ys, std::vector<float>* r_num)
    {
      float average_count = 0;
      for(int k=0 ; k<ys->center_size() ; k++)
      {
        int count = 0;
        for(int i=0 ; i<ys->size() ; i++)
        {
          if(ys->cluster(i) == k) count++;
        }
        r_num->push_back(count);
        average_count += count;
      }
      average_count /= ys->center_size();

      for(int k=0 ; k<ys->center_size() ; k++)
      {
        (*r_num)[k] = (*r_num)[k] - average_count;
      }
    }

    float tag_decoder_impl::poe_k(sample_information* ys, const std::vector<float> r_area, const std::vector<float> r_num, const int k)
    {
      float poe = r_area[k] + r_num[k];

      float conf = 0;
      int count = 0;
      for(int i=0 ; i<ys->size() ; i++)
      {
        if(ys->cluster(i) == k)
        {
          conf += max_value_pcluster_i(ys, i);
          count++;
        }
      }
      conf /= count;

      return poe / conf;
    }

    void tag_decoder_impl::clustering_error_detection(sample_information *ys)
    {
      while(!is_power_of_2(ys))
      {
        std::vector<float> poe;
        float max_poe = 0;
        float max_poe_id = -1;
        int max_poe_signal = 0;

        std::vector<float> r_area;
        calc_r_area(ys, &r_area);
        std::vector<float> r_num;
        calc_r_num(ys, &r_num);

        for(int k=0 ; k<ys->center_size() ; k++)
        {
          float current_poe = poe_k(ys, r_area, r_num, k);
          float current_signal;

          if(current_poe > 0) current_signal = 1;
          else current_signal = -1;
          poe.push_back(current_poe);
          current_poe = std::abs(current_poe);

          if(current_poe > max_poe)
          {
            max_poe = current_poe;
            max_poe_id = k;
            max_poe_signal = current_signal;
          }
        }

        if(max_poe_signal > 0)  // split
        {
          double second_max_decision = 0;
          int second_max_decision_id = -1;

          for(int i=0 ; i<ys->size() ; i++)
          {
            if(i == ys->center(max_poe_id)) continue;

            if(ys->cluster(i) == max_poe_id)
            {
              if(ys->decision(i) > second_max_decision)
              {
                second_max_decision = ys->decision(i);
                second_max_decision_id = i;
              }
            }
          }

          ys->push_back_center(second_max_decision_id);
          sample_clustering_after_splitting(ys, max_poe_id, ys->center_size()-1);
        }
        else  // merge
        {
          float min_poe = 3e38;
          int min_poe_id = -1;
          int x = 1;

          while(min_poe_id == -1)
          {
            for(int k=0 ; k<ys->center_size() ; k++)
            {
              if(max_poe_id == k) continue;

              if(IQ_distance(ys->sample(max_poe_id), ys->sample(k)) < x * CUTOFF_DISTANCE)
              {
                if(poe[k] < min_poe) min_poe = poe[k];
                min_poe_id = k;
              }
            }
            x *= 2;
          }

          ys->erase_center(max_poe_id);
          for(int i=0 ; i<ys->size() ; i++)
          {
            if(ys->cluster(i) == max_poe_id) ys->set_cluster(i, min_poe_id);
            if(ys->cluster(i) > max_poe_id) ys->decrease_cluster(i);
          }
        }

        print_cluster_sample(ys, "cluster_sample");
      }
    }

    void tag_decoder_impl::print_cluster_sample(sample_information* ys, const std::string filename)
    {
      std::ofstream file(filename, std::ios::app);

      file << "\t\t\t\t\t** center_id **" << std::endl;
      for(int i=0 ; i<ys->center_size() ; i++)
        file << ys->center(i) << " ";
      file << std::endl << std::endl;

      file << "\t\t\t\t\t** center(I) **" << std::endl;
      for(int i=0 ; i<ys->center_size() ; i++)
        file << ys->sample(ys->center(i)).real() << " ";
      file << std::endl;
      file << "\t\t\t\t\t** center(Q) **" << std::endl;
      for(int i=0 ; i<ys->center_size() ; i++)
        file << ys->sample(ys->center(i)).imag() << " ";
      file << std::endl << std::endl;

      for(int i=0 ; i<ys->center_size() ; i++)
      {
        file << "\t\t\t\t\t** cluster " << i << " (I) **" << std::endl;
        for(int j=0 ; j<ys->size() ; j++)
        {
          if(ys->cluster(j) == i) file << ys->sample(j).real() << " ";
        }
        file << std::endl;
        file << "\t\t\t\t\t** cluster " << i << " (Q) **" << std::endl;
        for(int j=0 ; j<ys->size() ; j++)
        {
          if(ys->cluster(j) == i) file << ys->sample(j).imag() << " ";
        }
        file << std::endl << std::endl;
      }

      file.close();
    }
  }
}
