/*******************************************************************************
**
** Filename:	spiasm.l
** {{{
** Project:	wbspi, a set of Serial Peripheral Interface cores
**
** Purpose:	Contains flex instructions to build a basic assembler for the
**		SPIASM language used by our SPI CPU (in ../rtl).
**
** Creator:	Dan Gisselquist, Ph.D.
**		Gisselquist Technology, LLC
**
********************************************************************************
** }}}
** Copyright (C) 2022, Gisselquist Technology, LLC
** {{{
** This program is free software (firmware): you can redistribute it and/or
** modify it under the terms of  the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or (at
** your option) any later version.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
** FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
** for more details.
**
** You should have received a copy of the GNU General Public License along
** with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
** target there if the PDF file isn't present.)  If not, see
** <http://www.gnu.org/licenses/> for a copy.
** }}}
** License:	GPL, v3, as defined and found on www.gnu.org,
* {{{
*		http://www.gnu.org/licenses/gpl.html
*
********************************************************************************
*
* }}}
*/

%{
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <ctype.h>

typedef struct SYMBOL_S {
	unsigned	addr;
	char		*str;
} SYMBOL;

typedef struct EQDEFN_S {
	unsigned	val;
	char		*str;
} EQDEFN;

#define	MAX_SYMBOLS	512
SYMBOL	symlist[MAX_SYMBOLS];
unsigned	nsyms = 0;

EQDEFN	defnlist[MAX_SYMBOLS];
unsigned	ndefns = 0;

extern "C" int yylex();
extern "C" int	yywrap() { return 1;}

#define	I_START		0
// #define	I_STOP		1
#define	I_READ		2
#define	I_SEND		4
#define	I_TXRX		6
#define	I_LAST		8
#define	I_HALT		10
#define	I_TICK		11
#define	I_TARGET	12
#define	I_CHANNEL	14
#define	I_NOOP		15
//
#define	I_STOP		16
#define	I_JUMP		17
#define	I_WAIT		18

int	posn = 0;
bool	m_debug = false;

void	addinsn(int);
void	addimm_lbl(const char *str);
void	addimm(int);
void	adddefn(const char *str);
void	label(const char *);
%}

%option yylineno
%option warn

%%

(?i:START)	{ addinsn(I_START); }
(?i:STOP)	{ addinsn(I_STOP); }
(?i:READ)	{ addinsn(I_READ); }
(?i:SEND)	{ addinsn(I_SEND); }
(?i:TXRX)	{ addinsn(I_TXRX); }
(?i:LAST)	{ addinsn(I_LAST); }
(?i:WAIT)	{ addinsn(I_WAIT); }
(?i:HALT)	{ addinsn(I_HALT); }
(?i:TICK)	{ addinsn(I_TICK); }
(?i:TGT)	{ addinsn(I_TARGET); }
(?i:TARGET)	{ addinsn(I_TARGET); }
(?i:JUMP)	{ addinsn(I_JUMP); }
(?i:CHAN)	{ addinsn(I_CHANNEL); }
(?i:CHANNEL)	{ addinsn(I_CHANNEL); }
(?i:NOP)	{ addinsn(I_NOOP); }
(?i:NOOP)	{ addinsn(I_NOOP); }
[A-Za-z_][A-Za-z_0-9]*[ \t]*=[ \t]*0[xX][0-9A-Fa-f] { adddefn(yytext);}
[A-Za-z_][A-Za-z_0-9]*[ \t]*=[ \t]*0[0-7]* { adddefn(yytext);}
[A-Za-z_][A-Za-z_0-9]*[ \t]*=[ \t]*[1-9][0-9]* { adddefn(yytext);}
[A-Za-z_][A-Za-z_0-9]*:	{ label(yytext); }
[A-Za-z_][A-Za-z_0-9]*  { addimm_lbl(yytext); }
[,]?[\s]*0[xX][0-9A-Fa-f]+ { addimm(strtoul(yytext,NULL,16));}
[,]?[\s]*0[0-7]+		  { addimm(strtoul(yytext,NULL, 8));}
[,]?[\s]*[1-9][0-9]*	  { addimm(strtoul(yytext,NULL,10));}
0		  { addimm(0); }
","		{ }
[ \t]+		{ }
";".*\n		{ }
"#".*\n	{ }
"//".*\n	{ }
\n { }
%%

bool	verbose_flag = false;
FILE	*hfile = NULL;		// C++ data file
// FILE	*dbgfp  = NULL;		// Debug file

const char	*INSN[] = {
		"START", "READ",  "SEND", "TXRX",
		"HALT",  "TARGET","JUMP", "NOOP"};

int	m_binsz, m_pos = 0, m_numimm = 0, m_lastpos = 0;
char	*m_binary = NULL;
int	m_last_insn = I_NOOP;
bool	m_imm = false, m_halted = false, m_active =false;

void	init_buffer(void) {
	// {{{
	m_binsz   = 512;
	m_binary  = new char[m_binsz];
	m_pos     = 0;
	m_lastpos = 0;
	m_numimm  = 0;
	m_imm     = false;
	m_halted  = false;
	m_active  = false;
}
// }}}

void	grow_buffer(void) {
	// {{{
	char *newbuf;

	assert(m_binsz >= 512);
	newbuf = new char[m_binsz * 2];
	memcpy(newbuf, m_binary, m_pos);
	m_binsz *= 2;
	delete[] m_binary;
	m_binary = newbuf;
}
// }}}

void	addinsn(int i) {
	// {{{
	if (verbose_flag) {
		// {{{
		fprintf(stderr, "Lastpos = %d\n", m_lastpos);
		if (m_pos > m_lastpos)
		switch((m_binary[m_lastpos] >> 4) & 0x0e) {
		case I_START:
			// {{{
			if (m_binary[m_lastpos] == 0x01f)
				fprintf(stderr, "> STOP\n");
			else
				fprintf(stderr, "> START %d\n", m_binary[m_lastpos] & 0x01f);
			break;
			// }}}
		case I_READ:
			fprintf(stderr, "> READ %2d\n", 1+(m_binary[m_lastpos] & 0x01f));
			break;
		case I_SEND:
			// {{{
			fprintf(stderr, "> SEND\t");
			for(unsigned k=0; k < (m_binary[m_lastpos] & 0x1f)+1;
									k++) {
				fprintf(stderr, " 0x%02x",
					m_binary[m_lastpos+k+1] & 0x0ff);
				if (k < (m_binary[m_lastpos] & 0x1f)) {
					fprintf(stderr, ",");
					if ((k & 7)==7)
						fprintf(stderr, "\n\t");
				}
			} fprintf(stderr, "\n");
			break;
			// }}}
		case I_TXRX:
			// {{{
			fprintf(stderr, "> TXRX\t");
			for(unsigned k=0; k < (m_binary[m_lastpos] & 0x1f)+1;
									k++) {
				fprintf(stderr, " 0x%02x",
					m_binary[m_lastpos+k+1] & 0x0ff);
				if (k < (m_binary[m_lastpos] & 0x1f)) {
					fprintf(stderr, ",");
					if ((k & 7)==7)
						fprintf(stderr, "\n\t");
				}
			} fprintf(stderr, "\n");
			break;
			// }}}
		case I_LAST:
			fprintf(stderr, "> LAST\n");
			break;
		case 0xa: // HALT/WAIT / TICK
			// {{{
			if (0xa0 == (m_binary[m_lastpos] & 0x0f8))
				fprintf(stderr, "> HALT\n");
			else if (0xa8 == (m_binary[m_lastpos] & 0x0f8))
				fprintf(stderr, "> WAIT\n");
			else
				fprintf(stderr, "> TICK\n");
			break;
			// }}}
		case 0xc: // TARGET/JUMP
			// {{{
			if (0xc0 == (m_binary[m_lastpos] & 0x0f8))
				fprintf(stderr, "> TARGET\n");
			else if (0xc8 == (m_binary[m_lastpos] & 0x0f8))
				fprintf(stderr, "> JUMP\n");
			else
				fprintf(stderr, "> ILL!\n");
			break;
			// }}}
		case 0xe: // CHAN / NOOP
			// {{{
			if (0xe0 == (m_binary[m_lastpos] & 0x0f0))
				fprintf(stderr, "> CHANNEL %d\n",
					m_binary[m_lastpos] & 0x0f);
			else
				fprintf(stderr, "> NOOP\n");
			break;
			// }}}
		default: break;	// Should never get here
		}

		fprintf(stderr, "%02d: ", m_pos);
		switch(i & 0x0f) {
			case I_START:
				fprintf(stderr, "Insn #%2d: START\n", i); break;
			case I_STOP:
				fprintf(stderr, "Insn #%2d: STOP\n", i); break;
			case I_READ:
				fprintf(stderr, "Insn #%2d: READ\n", i); break;
			case I_SEND:
				fprintf(stderr, "Insn #%2d: SEND\n", i); break;
			case I_TXRX:
				fprintf(stderr, "Insn #%2d: TXRX\n", i); break;
			case I_LAST:
				fprintf(stderr, "Insn #%2d: LAST\n", i); break;
			case I_HALT:
				fprintf(stderr, "Insn #%2d: HALT\n", i); break;
			case I_WAIT:
				fprintf(stderr, "Insn #%2d: WAIT\n", i); break;
			case I_TICK:
				fprintf(stderr, "Insn #%2d: TICK\n", i); break;
			case I_TARGET:
				fprintf(stderr, "Insn #%2d: TARGET\n", i); break;
			case I_JUMP:
				fprintf(stderr, "Insn #%2d: JUMP\n", i); break;
			case I_CHANNEL:
				fprintf(stderr, "Insn #%2d: CHANNEL\n", i); break;
			case I_NOOP:
				fprintf(stderr, "Insn #%2d: NOOP\n", i); break;
			default:
				fprintf(stderr, "Insn #%2d: (Unknwon)\n", i); break;
		}
		// }}}
	}

	if (i == I_STOP) {
		if (!m_active)
			// Two stops in a row are pointless
			return;
		m_binary[m_pos] = 0x01f;
		m_active = false;
	} else if (m_imm && ( ((m_binary[m_lastpos] & 0x0e0) >> 4) == i)
			&& m_numimm < 32 && (i== I_TXRX || i == I_SEND)) {
		// Do nothing.  No new instruction, we just continue the last
		// one
		if (verbose_flag) {
			fprintf(stderr, "Continuing I_TXRX|I_SEND\n");
		}
		return;
	} else {
		switch(i) {
		case I_START:
			if (m_active) {
				addinsn(I_STOP);
			} m_binary[m_pos] = 0;
			m_active = true;
			break;
		case I_READ:
			assert(m_active);
			m_binary[m_pos] = 0x20;
			break;
		case I_SEND:
			assert(m_active);
			m_binary[m_pos] = 0x40;
			break;
		case I_TXRX:
			assert(m_active);
			m_binary[m_pos] = 0x60;
			break;
		case I_LAST:
			m_binary[m_pos] = 0x80;
			break;
		case I_HALT:
			if (m_halted)
				return;
			m_binary[m_pos] = 0xa0;
			break;
		case I_TICK:
			m_binary[m_pos] = 0xb0;
			break;
		case I_TARGET:
			m_binary[m_pos] = 0xc0;
			break;
		case I_CHANNEL:
			assert(!m_active);
			m_binary[m_pos] = 0xe0;
			break;
		case I_NOOP:
			m_binary[m_pos] = 0xf0;
			break;
		case I_STOP:
			// Handled above, so should never come here
			m_binary[m_pos] = 0x1f;
			m_active = false;
			break;
		case I_JUMP:
			m_binary[m_pos] = 0xc8;
			break;
		case I_WAIT:
			m_binary[m_pos] = 0xa8;
			break;
		default: m_binary[m_pos] = 0x1f;
			m_active = false;
		}

		if (i == I_WAIT || i == I_HALT || i == I_JUMP)
			m_active = false;
	}

	m_numimm = 0;
	m_imm    = (i == I_TXRX) || (i == I_SEND);
	m_lastpos= m_pos;
	m_last_insn = i;

	m_pos ++;
	m_halted = (i== I_HALT);

	if (m_pos >= m_binsz)
		grow_buffer();
}
// }}}

void	addimm_lbl(const char *id) {
	// {{{
	unsigned	jk;

	for(jk=0; jk<ndefns; jk++) {
		if (strcmp(id, defnlist[jk].str) == 0) {
			addimm(defnlist[jk].val);
			return;
		}
	} fprintf(stderr, "ERR: %s not defined!\n", id);
}
// }}}

void	addimm(int imm) {
	// {{{
	if (verbose_flag)
		fprintf(stderr, "ADD-IMM: 0x%02x (INSN %d, IMM=%s)\n",
			imm & 0x0ff, m_last_insn, imm ? "true":"false");
	if (m_imm) {
		// {{{
		if (m_numimm != 0) {
			assert(m_last_insn == I_SEND
				|| m_last_insn == I_TXRX);
		}

		if (m_numimm == 32) {
			m_binary[m_lastpos] &= 0x0e0;
			m_binary[m_lastpos] |= 0x1f;
			addinsn(m_last_insn);
		}

		m_binary[m_pos++] = imm;
		m_numimm++;

		m_binary[m_lastpos] &= 0x0e0;
		m_binary[m_lastpos] |= (m_numimm - 1) & 0x1f;
		// }}}
	} else {
		// {{{
		unsigned	opcode;
		const char	*errcode;

		errcode = NULL;
		switch(m_last_insn) {
		case I_START:
			m_binary[m_lastpos] &= 0x0e0;
			m_binary[m_lastpos] |= (imm & 0x1f);
			m_imm = true;
			break;
		case I_READ:
			if (imm == 0) {
				fprintf(stderr, "ERR: Read immediate cannot be zero!\n");
			} else if(imm > 32) {
				m_binary[m_lastpos] &= 0xe0;
				m_binary[m_lastpos] |= (31 & 0x1f);

				while(imm >= 32) {
					addinsn(I_READ);
					addimm(32);
					imm -= 32;
				} if (imm > 0) {
					addinsn(I_READ);
					m_binary[m_lastpos] &= 0xe0;
					m_binary[m_lastpos] |= ((imm-1) & 0x1f);
				}
			} else {
				m_binary[m_lastpos] &= 0xe0;
				m_binary[m_lastpos] |= ((imm-1) & 0x1f);
			}
			m_imm = true;
			break;
		case I_LAST:
			errcode = "LAST";
			break;
		case I_WAIT:
			if (m_binary[m_lastpos] & 0x07)
				errcode = "WAIT";
			m_binary[m_lastpos] &= 0xf8;
			m_binary[m_lastpos] |= imm;
			break;
		case I_HALT:
			errcode = "HALT";
			break;
		case I_TARGET:
			errcode = "TARGET";
			break;
		case I_JUMP:
			errcode = "JUMP";
			break;
		case I_NOOP:
			errcode = "NOOP";
			break;
		default:
			errcode = "ILL";
			break;
		} if (errcode) {
			fprintf(stderr, "ERR: Command %s takes no immediates!\n",
				errcode);
		}
		// }}}
	}

	if (m_pos >= m_binsz)
		grow_buffer();
}
// }}}

void	dump(FILE *fp) {
	// {{{
	for(int p=0; p<m_pos; p++) {
		unsigned op = (m_binary[p] >> 4) & 0x0f;

		printf("%02x: ", p);

		switch(op) {
		case I_START: case 1:
			if (m_binary[p] == 0x01f)
				fprintf(fp, "  STOP\n");
			else
				fprintf(fp, "  START %d\n", (m_binary[p] & 0x1f));
			break;
		case I_READ: case I_READ+1:
			fprintf(fp, "  READ %d\n", 1+(m_binary[p] & 0x01f));
			break;
		case I_SEND: case I_SEND+1:
			fprintf(fp, "  SEND");
			for(unsigned k=0; k < (m_binary[p] & 0x01f)+1
					&& (k + p + 1 < m_pos); k++) {
				printf(" 0x%02x", m_binary[p+k+1] & 0x0ff);
				if (k < (m_binary[p] & 0x1f)) {
					printf(",");
					if ((k & 7)==7)
						printf("\n\t");
				}
			} printf("\n");
			p += (m_binary[p] & 0x01f)+1;
			break;
		case I_TXRX: case I_TXRX+1:
			fprintf(fp, "  TXRX");
			for(unsigned k=0; k < (m_binary[p] & 0x01f)+1
					&& (k + p + 1 < m_pos); k++) {
				printf(" 0x%02x", m_binary[p+k+1] & 0x0ff);
				if (k < (m_binary[p] & 0x1f)) {
					printf(",");
					if ((k & 7)==7)
						printf("\n\t");
				}
			}
			printf("\n");
			p += (m_binary[p] & 0x01f)+1;
			break;
		case I_LAST:
			printf("  LAST\n"); break;
		case I_HALT:
			if (0x08 == (m_binary[p] & 0x18)) {
				if (m_binary[p] & 0x07) {
					printf("  WAIT %d\n", m_binary[p] & 7);
				} else
					printf("  WAIT\n");
			} else if (0x00 == (m_binary[p] & 0x18)) {
				printf("  HALT\n"); break;
			} else
				printf("  TICK\n"); break;
			break;
		case I_TARGET:	if (m_binary[p] & 0x10)
				printf("  ILLEGAL!!! 0x%02x\n",
					m_binary[p]);
			else if(m_binary[p] & 0x08)
				printf("  JUMP\n");
			else
				printf("  TARGET\n");
			break;
		case I_CHANNEL:	if (0x0f0 == (m_binary[p] & 0x0f0))
				printf("  NOOP\n");
			else
				printf("  CHANNEL %d\n",
					m_binary[p] & 0x0f);
			break;
		default:	printf("  ILL\t(0x%x)\n", op); break;
		}
	}
}
// }}}

void	adddefn(const char *str) {
	// {{{
	char		*cpy, *ptr, *ptreq;
	unsigned	val;

	cpy = strdup(str);
	ptreq = ptr = strchr(cpy,'=');
	if (ptr == NULL) {
		free(cpy); return;
	}

	ptr++;
	while(*ptr && isspace(*ptr))
		ptr++;

	val = strtoul(ptr, NULL, 0);

	*ptreq = '\0';
	ptreq--;
	while(cpy < ptreq && isspace(*ptreq))
		*ptreq-- = '\0';

	defnlist[ndefns].val = val;
	defnlist[ndefns].str = strdup(cpy);
	ndefns++;
	assert(ndefns < MAX_SYMBOLS);

	free(cpy);
}
// }}}

void	label(const char *str) {
	// {{{
	if (m_pos > 0 && !m_halted) {
		addinsn(I_HALT);
	}

	if (verbose_flag) {
		fprintf(stderr, "New-Label: %s at %d\n", str, m_pos);
	}

	symlist[nsyms].addr = m_pos;
	symlist[nsyms].str  = strdup(str);

	unsigned slen = strlen(symlist[nsyms].str);
	if (symlist[nsyms].str[slen-1] == ':')
		// This should always be true
		symlist[nsyms].str[slen-1] = '\0';
	nsyms++;

	assert(nsyms < MAX_SYMBOLS);
}
// }}}

unsigned	filesz(FILE *fp) {
	// {{{
	unsigned long	here = ftell(fp), endp;

	if (0 != fseek(fp, 0l, SEEK_END)) {
		fprintf(stderr, "ERR: FILESZ(FSEEK)\n");
		perror("O/S Err:");
		exit(EXIT_FAILURE);
	}
	endp = ftell(fp);
	if (0 != fseek(fp, here, SEEK_SET)) {
		fprintf(stderr, "ERR: FILESZ(FSEEK return)\n");
		perror("O/S Err:");
		exit(EXIT_FAILURE);
	}
	return (unsigned)(endp-here);
}
// }}}

void	usage(void) {
	// {{{
	fprintf(stderr, ""
"Usage: spiasm [-hdv] [-o <outfile>] [infiles ...]\n"
"\n"
"\t-h\tThis usage statement\n"
"\t-c\tProduce a C file output, declaring a variable array\n"
"\t-d\tDisassemble the given file, rather than assembling it\n"
"\t-v\tVerbose mode (may or may not do anything)\n"
"\t-o <outfile>\tWrite the results to <outfile>.  If <outfile> is not"
"\t\tgiven, results will be written to standard out.\n"
"\t<infiles ...>\tA set of filenames, separated by spaces, to be either\n"
"\t\tassembled or (in the case of -d) disassembled.\n");
}
// }}}

int main(int argc, char **argv) {
	bool	dump_flag = false, cpp_flag = false;
	int	opt;
	// dbgfp  = fopen("dump.txt",  "w");

	init_buffer();

	int	nfiles = 0, argn;
	FILE	*finp, *fout = stdout;
	while(-1 != (opt = getopt(argc, argv, "hcdvo:"))) {
		// {{{
		switch(opt) {
		case 'h':
			usage();
			exit(EXIT_SUCCESS);
			break;
		case 'c':
			cpp_flag  = true;
			dump_flag = false;
			break;
		case 'd':
			dump_flag = true;
			cpp_flag  = false;
			break;
		case 'v':
			fprintf(stderr, "Verbose mode enabled\n");
			verbose_flag = true;
			break;
		case 'o':
			// output_file = strcpy(optarg);
			fout = fopen(optarg, "w");
			break;
		}
	}
	// }}}

	if (verbose_flag) {
		if (cpp_flag)
			fprintf(stderr, "Attempting to output in C++ format\n");
		if (dump_flag)
			fprintf(stderr, "Attempting to dump input files\n");
	}

	// Process individual files
	// {{{
	for(argn=optind; argn<argc; argn++) {
		if (verbose_flag)
			fprintf(stderr, "Attempting to read from %s\n", argv[argn]);
		finp = fopen(argv[argn], "r");
		if (finp == NULL) {
			fprintf(stderr, "ERR: Cannot open %s\n", argv[argn]);
			perror("O/S Err:");
			continue;
		}

		nfiles++;
		if (dump_flag) {
			// {{{
			unsigned	fln = filesz(finp);
			size_t		pos;

			if (m_binary) {
				delete[] m_binary;
				m_binsz = 0;
			}

			m_binary = new char[fln+1];
			m_binary[fln] = 0;
			m_pos = (pos = fread(m_binary, 1, fln, finp));
			fprintf(fout, "DUMP: %s\n===============================\n",
				argv[argn]);
			dump(fout);
			fprintf(fout, "\n");
			fclose(finp);
			delete[] m_binary;
			m_binary = NULL;
			// }}}
		} else {
			// yyin = yy_new_buffer(finp, 32768);
			yyin = finp;
			yylex();
		}
	}
	// }}}

	// (Optionally) process stdin
	// {{{
	if (nfiles != 0) {
		// {{{
		if (verbose_flag)
			fprintf(stderr, "All files processed\n");
		// }}}
	} else if (dump_flag) {
		// {{{
		size_t	nr;

		assert(m_binsz > 0);
		assert(m_binary != NULL);
		assert(m_pos == 0);

		while((nr = fread(&m_binary[m_pos], 1, m_binsz-m_pos, stdin))
				== m_binsz-m_pos) {

			if (m_binsz > 65536) {
				fprintf(stderr, "ERR: File size exceeds artificial 64kB limit!\n");
				exit(EXIT_FAILURE);
			}

			m_pos += nr;
			grow_buffer();
		}

		fprintf(fout, "DUMP: (stdin)\n====================\n");
		dump(fout);
		fprintf(fout, "\n");
		fclose(finp);
		delete[] m_binary;
		m_binary = NULL;
		// }}}
	} else {
		yylex();	// Use stdin
	}
	// }}}

	// Write the file out
	// {{{
	if (!dump_flag) {
		if (cpp_flag) {
			unsigned	sympos = 0, tabstart = 0;

			if (nsyms > 0 && symlist[0].addr == 0)
				fprintf(fout, "const char %s[] = {\n\t", symlist[sympos++].str);
			else
				fprintf(fout, "const char spiasm[] = {\n\t");
			for(int p=0; p<m_pos; p++) {
				if (sympos < nsyms && symlist[sympos].addr == p) {
					fprintf(fout, "};\n\nconst char %s[] = {\n\t", symlist[sympos++].str);
					tabstart = p;
				}

				fprintf(fout, "0x%02x", m_binary[p] & 0x0ff);
				if ((p==m_pos-1)
					||(sympos < nsyms
						&& symlist[sympos].addr==p+1))
					fprintf(fout, "\n");
				else if (((p-tabstart) & 7) != 7)
					fprintf(fout, ", ");
				else
					fprintf(fout, ",\n\t");
			} fprintf(fout, "};\n");
		} else {
			fwrite(m_binary, sizeof(char), m_pos, fout);
		}
	}
	// }}}

	fclose(fout); 
	return (EXIT_SUCCESS);
}

