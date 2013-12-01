#ifndef DIRDETECTOR__H
#define DIRDETECTOR__H


/**
 * @function dirDetector.h
 * @brief code read image data and do rotation work
 * @author xiaopeng chen
 */


using namespace cv;



/**
 *   @function read_line
 *   @brief read a line of string from file fp
 *   @param buf [out] buffer to store a line of string
 *   @param count [out] string length readed
 *   @param fp file handler to the file to be read
 *   @retval -1 error occurs
 *           >=0  length of read buffer
 */
int read_line(char * buf, int * count, FILE* fp);


/**
 *  @function get_word
 *  @brief to get a single word from buffer starting at start_index
 *  @param word [out] buffer to store a word
 *  @param wordlen [out] length of the retrived word
 *  @param buf [in] buffer where a word is read from
 *  @param start_index [inout] the starting index where a word is read, after a word is 
 *                             read, start_index move to start of the next word if possible
 *  @param buf_len [in] the length of buf
 *  @retval -1 error occurs
 *          >=0 length of the word
 *
 */

int get_word(char * word, int* word_len ,char * buf, int *start_index, int buf_len);


/**
 *  @function retrive_rotate_image
 *  @brief to rotate the image to horizontal state
 *  @param rotate_image [out] image that has been rotated
 *  @param ptheta [out] rotate angle from original image to rotate_image
 *  @param file_name [in] file name of input array data in txt format for an image
 *  @retval -1 error occurs
 *          0 success
 *
 */
int retrive_rotate_image( Mat & rotate_image, double * ptheta, char * file_name);


/**
 *  @function load_image
 *  @brief to load image from file path file_name to memory image 
 *  @param image [out] image that has been loaded
 *  @param file_name [in] file name of input array data in txt format for an image
 *  @retval -1 error occurs
 *          0 success
 *
 */
int load_image(Mat & image, char * file_name);


/**
 *  @function calc_rotate_image
 *  @brief to rotate image from binary image 
 *  @param rotate_image [out] memory to store rotated image
 *  @param ptheta [out] rotate angle in rad from original image
 *  @param origin_image [in] input original image
 *  @retval -1 error occurs
 *          0 success
 *
 */
int calc_rotate_image( Mat & rotate_image, double * ptheta, Mat & origin_image);



int add_template_info_to_rotated_image(Mat & terrian_image, double theta, Mat & original_image, const Mat & line_image, const Mat & zshape_image, const Mat& square_image  );


#endif










  
