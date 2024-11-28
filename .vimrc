" sudo apt install vim-gtk3 # to use clipboard
filetype plugin indent on


inoremap jj <Esc>
set number
set clipboard=unnamedplus

set shiftwidth=4
set tabstop=4
set expandtab
set softtabstop=4
set list listchars=tab:\▸\-

nnoremap <C-F5> :w<CR>:!python3 %<CR>

augroup python_indent
    autocmd!
    autocmd FileType python setlocal shiftwidth=4
    autocmd FileType python setlocal tabstop=4
    autocmd FileType python setlocal softtabstop=4
    autocmd FileType python setlocal expandtab
augroup END

augroup verilog_indent
    autocmd!
    autocmd FileType verilog,systemverilog setlocal shiftwidth=2
    autocmd FileType verilog,systemverilog setlocal tabstop=2
    autocmd FileType verilog,systemverilog setlocal softtabstop=2
    autocmd FileType verilog,systemverilog setlocal expandtab
augroup END

