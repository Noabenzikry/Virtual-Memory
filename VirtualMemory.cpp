//
// Created by Noa Benzikry on 16/07/2024.
//
#include "VirtualMemory.h"
#include "PhysicalMemory.h"
#include "MemoryConstants.h"
#include <cstdint>
#include <cmath>
#include <algorithm>


word_t cyclic_distance(word_t page1, word_t page2);


void clearFrame(uint64_t frameIndex) {
  // Write the empty page to each entry in the frame
  for (int offset = 0; offset < PAGE_SIZE; ++offset) {
    uint64_t physicalAddress = frameIndex * PAGE_SIZE + offset;
    PMwrite(physicalAddress, 0);
  }
}


void VMinitialize() {
  // Initialize by clearing frame 0 in physical memory
  clearFrame(0);
}


void updateFrameDetails(word_t currentFrame, uint64_t targetPage, uint64_t curPath,
                        word_t &maxFrame, word_t &evictPage, word_t &evictFrame, word_t &maxDistance) {
  maxFrame = std::max(maxFrame, currentFrame);
  word_t distance = cyclic_distance(static_cast<word_t>(targetPage), static_cast<word_t>(curPath));
  if (distance > maxDistance) {
    maxDistance = distance;
    evictPage = static_cast<word_t>(curPath);
    evictFrame = currentFrame;
  }
}

void traverseFrames(int depth, uint64_t targetPage, word_t curFrame, word_t parentFrame,
                    word_t &maxFrame, word_t &evictPage, word_t &evictFrame, word_t &maxDistance,
                    word_t &freeFrame, bool &foundFreeFrame, uint64_t curPath) {

  if (depth == TABLES_DEPTH) {
    updateFrameDetails(curFrame, targetPage, curPath, maxFrame, evictPage, evictFrame, maxDistance);
    return;
  }

  bool hasNonEmptyChild = false;
  for (int slot = 0; slot < PAGE_SIZE; ++slot) {
    word_t childFrame = 0;
    uint64_t address = curFrame * PAGE_SIZE + slot;
    PMread(address, &childFrame);

    if (childFrame != 0) {
      hasNonEmptyChild = true;
      uint64_t nextPath = (curPath << OFFSET_WIDTH) | slot;
      traverseFrames (depth
                      + 1, targetPage, childFrame, parentFrame, maxFrame, evictPage, evictFrame, maxDistance, freeFrame, foundFreeFrame, nextPath);
    }
  }

  if (!hasNonEmptyChild && curFrame != parentFrame) {
    if (!foundFreeFrame || curFrame < freeFrame) {
      freeFrame = curFrame;
      foundFreeFrame = true;
    }
  }

  maxFrame = std::max(maxFrame, curFrame);
}


void evictFrame(word_t targetFrame, word_t curFrame, int currentDepth, bool &targetFound) {
  if (currentDepth == TABLES_DEPTH) {
    return; // Base case: reached the deepest level
  }

  for (int i = 0; i < PAGE_SIZE; ++i) {
    word_t childFrame;
    uint64_t address = curFrame * PAGE_SIZE + i;
    PMread(address, &childFrame);

    if (childFrame == targetFrame) {
      // If the target frame is found, clear it and mark it as found
      PMwrite(address, 0);
      targetFound = true;
      return;
    } else if (childFrame != 0) {
      // If there is a non-zero child, continue searching
      evictFrame(targetFrame, childFrame, currentDepth + 1, targetFound);
      if (targetFound) {
        return;
      }
    }
  }
}


word_t cyclic_distance(word_t page1, word_t page2)
{
  return std::min((int)NUM_PAGES - std::abs((int)page1 - (int)page2), std::abs((int)page1 - (int)page2));
}


void allocateAndInitFrame(uint64_t virtualAddress, uint64_t currFrame, word_t &frameIndex) {
  bool foundFreeFrame = false;
  word_t freeFrame = NUM_FRAMES;
  word_t highestFrame = 0;
  word_t evictPage = 0;
  word_t maxFrame = 0;
  word_t maxDist = 0;

  // Traverse frames and find a free frame or determine which frame to evict
  traverseFrames(0, virtualAddress >> OFFSET_WIDTH, 0, frameIndex, highestFrame, evictPage, maxFrame, maxDist, freeFrame, foundFreeFrame, 0);

  // If a free frame was found, allocate it
  if (foundFreeFrame) {
    frameIndex = freeFrame;
    bool targetFound = false;
    evictFrame(freeFrame, 0, 0, targetFound);
    clearFrame(freeFrame); // Initialize the new frame
  } else {
    // If no free frame is available, use the highest frame or evict
    if (highestFrame < NUM_FRAMES - 1) {
      frameIndex = highestFrame + 1;
      clearFrame(frameIndex); // Initialize the new frame
    } else {
      PMevict(maxFrame, evictPage);
      bool targetFound = false;
      evictFrame(maxFrame, 0, 0, targetFound);
      frameIndex = maxFrame;
      clearFrame(frameIndex); // Initialize the new frame
    }
  }

  PMwrite(currFrame, frameIndex);
}



uint64_t convertAddress(uint64_t virtualAddress) {
  word_t frameIndex = 0;
  word_t newAddress = 0;

  for (int depth = 0; depth < TABLES_DEPTH; depth++) {
    uint64_t currFrame = (PAGE_SIZE * frameIndex) +
        ((virtualAddress >>(OFFSET_WIDTH *(TABLES_DEPTH - depth)))
        & ((1ULL << OFFSET_WIDTH) - 1));

    PMread(currFrame, &newAddress);

    if (newAddress == 0) {
      allocateAndInitFrame(virtualAddress, currFrame, frameIndex);
    } else {
      frameIndex = newAddress;
    }
  }

  PMrestore(frameIndex, virtualAddress >> OFFSET_WIDTH);
  uint64_t pageOffset = virtualAddress % ((1ULL << OFFSET_WIDTH));
  uint64_t physicalAddress = frameIndex * PAGE_SIZE + pageOffset;
  return physicalAddress;
}

int VMread(uint64_t virtualAddress, word_t* value) {
  if ((virtualAddress >= VIRTUAL_MEMORY_SIZE) || (value == nullptr))
  {
    return 0;
  }

  uint64_t addr = convertAddress(virtualAddress);
  if (addr == 0) {
    return 0;
  }

  PMread(addr, value); // Second translation
  return 1; // Success
}

int VMwrite(uint64_t virtualAddress, word_t value) {
  if (virtualAddress >= VIRTUAL_MEMORY_SIZE)
  {
    return 0;
  }

  uint64_t addr = convertAddress(virtualAddress);
  if (addr == 0) {
    return 0;
  }

  PMwrite(addr, value); // Second translation
  return 1; // Success
}
