# Converted Documentation

This folder contains documentation that has been converted from DokuWiki format to Markdown using pandoc.

## Source
- Original DokuWiki files: `design-documents/wiki/manual/data/pages/`
- Conversion date: September 24, 2025

## Conversion Process
All `.txt` files containing DokuWiki markup were converted to `.md` Markdown files using:
```bash
pandoc -f dokuwiki -t markdown input.txt -o output.md
```

## Files Converted
A total of 35 files were converted, including:

### Main Documentation
- `introduction.md` - Introduction to Frankfurt RS-232
- `getting_started.md` - Getting started guide
- `hardware.md` - Hardware documentation
- `installing_the_module.md` - Installation instructions
- `can4vscp_cabling.md` - Cabling information
- `data_coding.md` - Data coding specifications
- `decision_matrix.md` - Decision matrix documentation
- `faq.md` - Frequently asked questions
- `change_log_1.md` - Change log

### Operating Modes
- `the_can4vscp_mode.md` - CAN4VSCP mode
- `the_slcan_mode.md` - SLCAN mode  
- `the_verbose_mode.md` - Verbose mode

### Usage Guides
- `using_with_beaglebone.md` - BeagleBone usage
- `using_with_carambola.md` - Carambola usage
- `using_with_raspberry_pi.md` - Raspberry Pi usage
- `using_with_software.md` - Software usage
- `using_with_vscp_daemon.md` - VSCP daemon usage
- `using_with_vscp_works.md` - VSCP Works usage

### Technical Documentation
- `vscp_boot_loader_algorithm.md` - Boot loader algorithm
- `vscp_level_i_specifics.md` - VSCP Level I specifics
- `vscp_multicast.md` - VSCP multicast
- `level_i_events.md` - Level I events
- `level_ii_events.md` - Level II events
- `level_ii_specifics.md` - Level II specifics
- `physical_level_lower_level_protocols.md` - Physical level protocols
- `register_abstraction_model.md` - Register abstraction model
- `module_description_file.md` - Module description file
- `globally_unique_identifiers.md` - Globally unique identifiers
- `replacing_the_firmware.md` - Firmware replacement

### Miscellaneous
- `start.md` - Start page
- `download_page.md` - Download page
- `playground/playground.md` - Playground content
- `wiki/dokuwiki.md` - DokuWiki information
- `wiki/syntax.md` - Wiki syntax
- `wiki/welcome.md` - Welcome page

## Notes
- Image references have been converted but may need path adjustments
- Some DokuWiki-specific markup may require manual review
- Internal links between pages may need to be updated to reference `.md` files instead of `.txt`