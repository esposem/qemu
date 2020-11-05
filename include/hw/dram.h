
#ifndef HW_DRAM_H
#define HW_DRAM_H

#define DRAM_MAX_BIT_INTERLEAVING 5
typedef struct dram_element_info {
    uint64_t mask;
    uint8_t bits[DRAM_MAX_BIT_INTERLEAVING];
    int8_t offsets[DRAM_MAX_BIT_INTERLEAVING]; // offset abs to bit 0
    uint8_t n_sections;
    uint64_t size;
} dram_element_info;

typedef struct dram_cpu_info {
    dram_element_info channel;
    dram_element_info rank;
    dram_element_info bank;
    dram_element_info row;
    dram_element_info subarr;
    dram_element_info col;
    uint64_t offset;
    uint64_t size;

    // just to avoid doing it every time
    uint64_t part_row_start[DRAM_MAX_BIT_INTERLEAVING -1]; // LSB of each part of the mask
    uint64_t part_row_end; // portion of the mask nearest to the LSB
} dram_cpu_info;



/*
    8 banks
    2^11 column per row
    2^14 rows
    1 channel

    Cache row interleaving (cache 64 bytes)
    [31] 		channel     0x8000 0000
    [30] 		rank        0x4000 0000
    [29:24]     subarr      0x3f00 0000
    [29:15]		row         0x3fff 8000
    [14:9]		high col    0x0000 7e00
    [8:6]		bank        0x0000 01c0
    [5:0]		low col     0x0000 003f

    ROW Interleaving
    [31] 		channel
    [30] 		rank
    [29:24]     subarr
    [29:15]		row
    [14:12]		bank
    [11:0]		col
*/

static inline void print_field(const char *name, int8_t *el)
{
    printf("%s:", name);
    for(int i=0; i < DRAM_MAX_BIT_INTERLEAVING; i++){
        printf("%d-", el[i]);
    }
    printf(" ");
}

static inline void print_dram_element(const char *name, dram_element_info *el)
{
    printf("%s: ", name);
    print_field("bits",(int8_t *) el->bits);
    print_field("off", el->offsets);
    printf("mask:%lx ", el->mask);
    printf("size:%ld\n", el->size);
}

static inline void init_default_dram_elements(dram_cpu_info *dram)
{
    dram->channel.n_sections = 1;
    dram->channel.bits[0] = 1;
    dram->channel.mask = 0x100000000;

    dram->rank.n_sections = 1;
    dram->rank.bits[0] = 1;
    dram->rank.mask = 0x80000000;

    dram->row.n_sections = 1;
    dram->row.bits[0] = 15;
    dram->row.mask = 0x7fff8000;

    dram->subarr.n_sections = 1;
    dram->subarr.bits[0] = 6;
    dram->subarr.mask = 0x7f000000;

    dram->bank.n_sections = 1;
    dram->bank.bits[0] = 3;
    dram->bank.mask = 0x00007000;

    dram->col.n_sections = 1;
    dram->col.bits[0] = 12;
    dram->col.mask = 0x00000fff;
}

static inline int parse_dram_el_bits(dram_element_info *el)
{
    char *f, *p, *f1, *f2, *p1, *p2;
    int sf, ef, sp, ep, shift_off;
    uint32_t new_bits;
    f = strtok(NULL, "="); /* f1:f2 */
    p = strtok(NULL, "\n"); /* p1:p2 */

    g_assert(f != NULL && p != NULL);

    f1 = strtok(f, ":");
    f2 = strtok(NULL, "\n");
    p1 = strtok(p, ":");
    p2 = strtok(NULL, "\n");

    sf = atoi(f1);
    sp = atoi(p1);

    // check that the addresses match
    if((!f2 && p2) || (!p2 && f2)){
        return 1;
    }

    if(f2){
        ef = atoi(f2);
        ep = atoi(p2);
        new_bits = abs(sf-ef) + 1;

        // check bit size match
        if(new_bits != (abs(sp-ep) + 1))
            return 1;

        shift_off = MIN(sp, ep);

        el->mask |= ((1 << new_bits) - 1) << shift_off;

        // shift_off -= MIN(sf, ef);
    } else {
        new_bits = 1;
        shift_off = sp;
        el->mask |= 1l << shift_off;
        // shift_off -= (sf+1);
    }
    el->bits[el->n_sections] = new_bits;
    new_bits = 0;

    el->offsets[el->n_sections] = shift_off;
    el->n_sections++;

    for(int i=0; i < el->n_sections; i++)
        new_bits += el->bits[i];
    el->size = (1 << new_bits);

    return 0;
}

static inline void read_dram_info_file(dram_cpu_info *dram_info)
{
    FILE *fi;
    char buff[100];
    char *op;

    memset(dram_info, 0, sizeof(dram_cpu_info));

    fi = fopen("dram_struct.txt", "r");
    if(fi == NULL){
        perror("fopen");
        printf("initializing to default DRAM params\n");
        init_default_dram_elements(dram_info);
        return;
    }

    while(fgets(buff, 100, fi) != NULL) {
        op = strtok(buff, " ");
        if(op == NULL || op[0] == '\n' || op[0] == '#')
            continue;

        dram_element_info *el = NULL;
        switch(op[0]) {
            case 'c': {
                if(op[1] == 'o') // col
                    el = &dram_info->col;
                else // chan
                    el = &dram_info->channel;
                break;
            }
            case 'r': {
                    if(op[1] == 'o') // row
                        el = &dram_info->row;
                    else if (op[1] == 'a')
                        el = &dram_info->rank;
                    else if (op[1] == 'e') {
                        char *addr = strtok(NULL, " ");
                        char *sz = strtok(NULL, "\n");
                        dram_info->offset = strtoull(addr, NULL, 16);
                        dram_info->size = strtoull(sz, NULL, 16);
                    }

                break;
            }
            case 's':
                    el = &dram_info->subarr;
                break;

            case 'b':
                    el = &dram_info->bank;
                break;

            default:
                printf("Line '%s' not recognized. Ignoring\n", buff);
                break;
        }

        if(el){
            if(parse_dram_el_bits(el)) {
                printf("Line '%s' not correctly formed. Ignoring\n", buff);
            }
            // print_dram_element(op, el);
        }
    }

    print_dram_element("col", &dram_info->col);
    print_dram_element("bank", &dram_info->bank);
    print_dram_element("row", &dram_info->row);
    print_dram_element("subarr", &dram_info->subarr);
    print_dram_element("rank", &dram_info->rank);
    print_dram_element("channel", &dram_info->channel);
    printf("addr %lx sz %lx\n", dram_info->offset, dram_info->size);
    printf("---------------------\n");

}

#endif