/*
 * test_kdtree.cpp
 *
 *  Created on: 27.03.2013
 *      Author: josh
 */




#include <gtest/gtest.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <cob_3d_feature_map/libkdtree/kdtree++/kdtree.hpp>
#include <cob_3d_feature_map/cluster.h>
#include <cob_3d_feature_map/representation.h>
#include <cob_3d_feature_map/feature.h>
#include <cob_3d_feature_map/instance.h>
#include <cob_3d_feature_map/haar.h>
#include <cob_3d_feature_map/correspondence.h>

using namespace std;

template<typename FT>
void fill_random(FT &node) {
	for(int i=0; i<node.cols(); i++)
		for(int j=0; j<node.rows(); j++)
			node(j,i) = (rand()%1000-344)/100.;
}

TEST(ft_map, test_2d)
{
	boost::mt19937 rng;
	boost::normal_distribution<> nd(0., .1);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);

	typedef cob_3d_feature_map::Feature_kdtree<Eigen::Vector2f,2,float> FT;
	typedef cob_3d_feature_map::ClusterReprsentation<2> CR;
	typedef cob_3d_feature_map::Instance<CR, FT> INST;
	typedef cob_3d_feature_map::Cluster<INST> CL;
	typedef cob_3d_feature_map::CorrespondenceFinding::Correspondence<INST, CR> COR;
	typedef cob_3d_feature_map::CorrespondenceSet<CL,COR> COR_SET;

	vector<boost::shared_ptr<CL> > clusters;

	typedef KDTree::KDTree<FT> treeType;
	treeType tree;

	// generate random clusters
	for ( size_t m = 0; m < 101; ++m) {

		//generate similar clusters
		if((m>=2&&m<=5)||m==100) {
			boost::shared_ptr<CL> pcl(new CL);
			clusters.push_back(pcl);
			for ( size_t n = 0; n < clusters[0]->getInstances().size(); ++n)
			{
				CR pt;
				Eigen::Vector2f v = (*clusters[0]->getInstances()[n]).getRepresentation().getMean();
				v(0)+=var_nor();//(rand()%11-5)/20.;
				v(1)+=var_nor();//(rand()%11-5)/20.;	//NOT normal distributed: hihi
				pt+=v;

				boost::shared_ptr<INST> inst(new INST(pt));
				(*clusters.back()) += inst;
			}

			for(size_t l=0; l<m%11; l++) {
				CR pt;
				Eigen::Vector2f v;
				fill_random(v);
				pt+=v;

				boost::shared_ptr<INST> inst(new INST(pt));
				(*clusters.back()) += inst;
			}
		}
		else if(m==1) {
			boost::shared_ptr<CL> pcl(new CL);
			clusters.push_back(pcl);
			for ( size_t n = 0; n < clusters[0]->getInstances().size(); ++n)
			{
				CR pt;
				Eigen::Vector2f v = (*clusters[0]->getInstances()[n]).getRepresentation().getMean();
				pt+=v;

				boost::shared_ptr<INST> inst(new INST(pt));
				(*clusters.back()) += inst;
			}
		}
		else {
			boost::shared_ptr<CL> pcl(new CL);
			clusters.push_back(pcl);
			for ( size_t n = 0; n < 40; ++n)
			{
				CR pt;
				Eigen::Vector2f v;
				fill_random(v);
				pt+=v;

				boost::shared_ptr<INST> inst(new INST(pt));
				(*clusters.back()) += inst;
			}
		}

		//generate features
		for(size_t i=0; i<clusters.back()->getInstances().size(); i++)
		{
			std::vector<float> dist;
			for(size_t j=0; j<clusters.back()->getInstances().size(); j++) {
				if(i==j) continue;
				dist.push_back( (clusters.back()->getInstances()[i]->getRepresentation().getMean()-clusters.back()->getInstances()[j]->getRepresentation().getMean()).norm() );
			}
			std::sort(dist.begin(),dist.end());

			for(size_t k=0; k<3*(FT::DIMENSION*(FT::DIMENSION+1))/2; k++) {
				boost::shared_ptr<FT> ft(new FT);
				(*ft)[0] = dist[k]+dist[k+30]+dist[k+20]+dist[k+10];
				if(m!=0&&i<clusters[0]->getInstances().size() && (m<=5||m>=100)) {
					(*ft)[1] = (*clusters[0]->getInstances()[i]->getFeatures()[k])[1];
				} else {
					if(k==0)
						(*ft)[1] = (std::min(rand()%3,2))/2.;
					else
						(*ft)[1] = (*clusters.back()->getInstances()[i]->getFeatures()[0])[1];
				}
				//(*ft)[1] = dist[k+1];
				tree.insert(*ft);
				clusters.back()->getInstances()[i]->append(ft);
			}
		}
	}

	//build descriptor
	for(size_t c=0; c<clusters.size(); c++) {
		for(size_t i=0; i<clusters[c]->getInstances().size(); i++)
		{
			// now do the same with the kdtree.
			for(size_t j=0; j<clusters[c]->getInstances()[i]->getFeatures().size(); j++) {
				vector<FT> howClose;
				vector< vector<float> > descr;
				tree.find_within_range( *clusters[c]->getInstances()[i]->getFeatures()[j],0,back_insert_iterator2<vector<FT> >(howClose),back_insert_iterator2<vector<vector<float> > >(descr));
				clusters[c]->getInstances()[i]->getFeatures()[j]->setDescriptorFromVector(descr[0]);
				clusters[c]->getInstances()[i]->updateDescriptor(
						clusters[c]->getInstances()[i]->getFeatures()[j]
				);
			}
		}
	}

	//debug svg
	for(size_t c=0; c<clusters.size(); c++) {
		char fn[512];
		sprintf(fn,"/tmp/cluster%d.svg",(int)c);
		FILE *fp=fopen(fn,"w");
		fputs("<?xml version=\"1.0\" ?><svg width=\"200\" height=\"200\">",fp);
		for(size_t i=0; i<clusters[c]->getInstances().size(); i++)
		{
			sprintf(fn,"<circle cx=\"%f\" cy=\"%f\" r=\"3\" />",
					clusters[c]->getInstances()[i]->getRepresentation().getMean()(0)*20 + 344/100.f,
					clusters[c]->getInstances()[i]->getRepresentation().getMean()(1)*20 + 344/100.f);
			fputs(fn,fp);
		}
		fputs("</svg>",fp);
		fclose(fp);
	}

        //boost::shared_ptr<CL> search_cluster;
	//search_cluster = clusters[100];
	COR_SET search_cluster(clusters[100]);
	clusters.resize(100);

	std::vector<COR_SET> sets;
	std::vector<float> result;
	for(vector<boost::shared_ptr<CL> >::const_iterator it = clusters.begin()+0; it!=clusters.end(); ++it)
	  sets.push_back( COR_SET(*it, search_cluster.getPtr()) );
	size_t p = cob_3d_feature_map::haar_wavelet(result, sets, search_cluster, 0.7f);

	for(size_t i=0; i<result.size(); i++)
		std::cout<<"R: "<<result[i]<<std::endl;
	std::cout<<"best match: "<<p<<std::endl;



        //debug svg
        //size_t c=0; //or "p"
	for(size_t c=0; c<5; c++) {
                char fn[512];
                sprintf(fn,"/tmp/comp%d.svg",(int)c);
                FILE *fp=fopen(fn,"w");
                fputs("<?xml version=\"1.0\" ?><svg width=\"400\" height=\"200\">",fp);
                for(size_t i=0; i<search_cluster->getInstances().size(); i++)
                {
                        sprintf(fn,"<circle cx=\"%f\" cy=\"%f\" r=\"3\" fill=\"black\"/>",
                                        search_cluster->getInstances()[i]->getRepresentation().getMean()(0)*20 + 344/100.f,
                                        search_cluster->getInstances()[i]->getRepresentation().getMean()(1)*20 + 344/100.f);
                        fputs(fn,fp);
                }
                for(size_t i=0; i<clusters[c]->getInstances().size(); i++)
                {
                        sprintf(fn,"<circle cx=\"%f\" cy=\"%f\" r=\"3\" fill=\"red\"/>",
                                        clusters[c]->getInstances()[i]->getRepresentation().getMean()(0)*20 + 344/100.f+200,
                                        clusters[c]->getInstances()[i]->getRepresentation().getMean()(1)*20 + 344/100.f);
                        fputs(fn,fp);
                }
                for(size_t i=0; i<sets[c].getCors().size(); i++) {
                  if(sets[c].getCors()[i].getMaxP()<0.5) continue;
                  sprintf(fn,"<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"stroke:rgb(0,0,0);stroke-width:1\"/>",
                          sets[c].getCors()[i].getOrigin()->getRepresentation().getMean()(0)*20 + 344/100.f+200,
                          sets[c].getCors()[i].getOrigin()->getRepresentation().getMean()(1)*20 + 344/100.f,
                          sets[c].getCors()[i].getBestMatch()->getRepresentation().getMean()(0)*20 + 344/100.f,
                          sets[c].getCors()[i].getBestMatch()->getRepresentation().getMean()(1)*20 + 344/100.f);
                  fputs(fn,fp);
                }
                fputs("</svg>",fp);
                fclose(fp);
        }
}


int main(int argc, char **argv){
	//ros::Time::init();
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

