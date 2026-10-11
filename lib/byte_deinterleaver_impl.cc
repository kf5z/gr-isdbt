/* -*- c++ -*- */
/*  
 * Copyright 2015, 2016, 2017, 2018, 2019, 2020, 2021
 *   Federico "Larroca" La Rocca <flarroca@fing.edu.uy>
 *   Pablo Belzarena 
 *   Gabriel Gomez Sena 
 *   Pablo Flores Guridi 
 *   Victor Gonzalez Barbone
 *
 *   Instituto de Ingenieria Electrica, Facultad de Ingenieria,
 *   Universidad de la Republica, Uruguay.
 *  
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *  
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "byte_deinterleaver_impl.h"
#include <stdio.h>
#include <algorithm>

namespace gr {
    namespace isdbt {

        const int byte_deinterleaver_impl::d_SYNC = 0x47;
        const int byte_deinterleaver_impl::d_TSP_SIZE = 204; 
        const int byte_deinterleaver_impl::d_I = 12; 
        const int byte_deinterleaver_impl::d_M = 17; 

        byte_deinterleaver::sptr
            byte_deinterleaver::make()
            {
                return gnuradio::get_initial_sptr
                    (new byte_deinterleaver_impl());
            }

        /*
         * The private constructor
         */
        byte_deinterleaver_impl::byte_deinterleaver_impl()
            : gr::block("byte_deinterleaver",
                    gr::io_signature::make(1, 1, sizeof(unsigned char)),
                    gr::io_signature::make(1,1,sizeof(unsigned char)*d_TSP_SIZE))
        {

            // this block will output a vector of size d_TSP_SIZE
            set_relative_rate(1.0/(d_TSP_SIZE)); 
            
            // The "difficult" part in any deinterleaver is setting the buffer's
            // size right. Here, we put them in the reverse order of the transmitter. 
            for (int i=d_I-1; i>=0; i--)
                d_shift.push_back(new boost::circular_buffer<unsigned char>(1 + d_M*i,0)); 
                //d_shift.push_back(new std::deque<unsigned char>(d_M*i,0)); 

            set_tag_propagation_policy(gr::block::TPP_DONT); 
        }

        /*
         * Our virtual destructor.
         */
        byte_deinterleaver_impl::~byte_deinterleaver_impl()
        {
            for (auto* shift : d_shift)
                delete shift;
        }

        void
            byte_deinterleaver_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
            {
                int ninputs = ninput_items_required.size ();

                // this block will output a vector of d_TSP_SIZE bytes, and will thus require this amount of 
                // bytes in its input to generate a single output
                for (int i = 0; i < ninputs; i++){
                    ninput_items_required[i] = noutput_items * d_TSP_SIZE;
                    //printf("ninput_items_required[%i]=%i\n",i,ninput_items_required[i]); 
                }
            }

        int
            byte_deinterleaver_impl::general_work (int noutput_items,
                    gr_vector_int &ninput_items,
                    gr_vector_const_void_star &input_items,
                    gr_vector_void_star &output_items)
            {
                const unsigned char *in = (const unsigned char *) input_items[0];
                unsigned char *out = (unsigned char *) output_items[0];

                const int available_packets = std::min(noutput_items,
                                                        ninput_items[0] / d_TSP_SIZE);
                const uint64_t nread = nitems_read(0);
                std::vector<tag_t> tags;
                get_tags_in_range(tags, 0, nread,
                                  nread + available_packets * d_TSP_SIZE,
                                  pmt::string_to_symbol("frame_begin"));
                std::sort(tags.begin(), tags.end(), [](const tag_t& a, const tag_t& b) {
                    return a.offset < b.offset;
                });

                int d_to_consume = available_packets * d_TSP_SIZE;
                size_t at_start = 0;
                while (at_start < tags.size() && tags[at_start].offset == nread)
                    ++at_start;
                // Leave the next boundary for a separate work call, even when
                // this call starts on a frame boundary. Partial packets before
                // resynchronization are discarded, as in the original block.
                if (at_start < tags.size())
                    d_to_consume = static_cast<int>(tags[at_start].offset - nread);
                const int d_to_out = d_to_consume / d_TSP_SIZE;
                if (d_to_out > 0) {
                    for (size_t i = 0; i < at_start; ++i) {
                        add_item_tag(0, nitems_written(0),
                                     pmt::string_to_symbol("frame_begin"),
                                     pmt::string_to_symbol("generated by the byte deinterleaver"));
                    }
                }

                //PRINTF("DEINTERLEAVER: noutput_items: %i\n", noutput_items);

                int index_out = 0; 
                for (int i = 0; i < d_to_out; i++)
                {
                    for (int byte_index = 0; byte_index<d_TSP_SIZE; byte_index++)
                    {
                        index_out = byte_index + i*d_TSP_SIZE; 
                        d_shift[index_out % d_I]->push_back(in[index_out]);
                        out[index_out] = d_shift[index_out % d_I]->front();
                        d_shift[index_out % d_I]->pop_front();
                        //printf("DEINTERLEAVER: in[%i]=%x; out[%i]=%x\n", index_out, in[index_out],index_out, out[index_out]);
                        //if (index_out % 204 == 204-1)
                         //   printf("DEINTERLEAVER: out[%i]=%x\n", index_out, out[index_out]);
                    }
                }

                //PRINTF("DEINTERLEAVER: to_out: %i\n", to_out);

                // Tell runtime system how many input items we consumed on
                // each input stream.
                consume_each(d_to_consume);

                // Tell runtime system how many output items we produced.
                return (d_to_out);
            }

    } /* namespace isdbt */
} /* namespace gr */

