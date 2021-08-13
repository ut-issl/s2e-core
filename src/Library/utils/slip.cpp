
#include "slip.h"

#include <algorithm>
#include <iterator>

static uint8_t kSlipFend_  = 0xc0;
static uint8_t kSlipFesc_  = 0xdb;
static uint8_t kSlipTfend_ = 0xdc;
static uint8_t kSlipTfesc_ = 0xdd;

std::vector<uint8_t> decode_slip(const std::vector<uint8_t> in)
{
  std::vector<uint8_t> out = in;
  // Footer Search
  auto fend_itr = std::find(out.begin(), out.end(), kSlipFend_);
  out.erase(fend_itr, out.end());
  
  // FESC search
  auto fesc_itr = out.begin();
  while(1)
  {
    fesc_itr = std::find(fesc_itr, fend_itr, kSlipFesc_);
    if (fesc_itr == fend_itr) break;
    // convert
    if (*(fesc_itr + 1) == kSlipTfend_)
    {
      *(fesc_itr + 1) = kSlipFend_;
    }
    else if(*(fesc_itr + 1) == kSlipTfesc_)
    {
      *(fesc_itr + 1) = kSlipFesc_;
    }
    else
    {
      // TODO error handling
    }
    fesc_itr = out.erase(fesc_itr);
    fesc_itr++;
  }

  return out;
}

std::vector<uint8_t> encode_slip(const std::vector<uint8_t> in)
{
  std::vector<uint8_t> out = in;
  
  // FESC search
  auto fesc_itr = out.begin();
  while(1)
  {
    fesc_itr = std::find(fesc_itr, out.end(), kSlipFesc_);
    if (fesc_itr == out.end()) break;
    // convert
    fesc_itr = out.insert(fesc_itr+1, kSlipTfesc_);
  }

  // FEND search
  auto fend_itr = out.begin();
  while(1)
  {
    fend_itr = std::find(fend_itr, out.end(), kSlipFend_);
    if (fend_itr == out.end()) break;
    // convert
    *fend_itr = kSlipFesc_;
    fend_itr = out.insert(fend_itr+1, kSlipTfend_);
  }

  out.push_back(kSlipFend_);
  return out;
}
