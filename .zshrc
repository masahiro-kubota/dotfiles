
# ~/.zshrc: executed by zsh for interactive shells.

# インタラクティブシェルでない場合は終了
if [[ $- != *i* ]]; then
  return
fi


# プロンプトの設定
PROMPT='%F{green}%n@%m%f:%F{blue}%~%f$ '

# カラー設定
if [ -x /usr/bin/dircolors ]; then
    [[ -r ~/.dircolors ]] && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
fi

# エイリアス
if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

alias -g G='|grep'

# パスと環境変数
export PATH="$PATH:/home/masa/.local/bin"
export CCACHE_DIR="/var/tmp/ccache"
export CC="/usr/lib/ccache/gcc"
export CXX="/usr/lib/ccache/g++"
. "$HOME/.cargo/env"

# Zsh 用補完機能の読み込み
autoload -U compinit
compinit

# history begging search 前方一致限定の履歴検索
bindkey "^P" history-beginning-search-backward
bindkey "^N" history-beginning-search-forward

# インクリメンタル検索のバインド
stty -ixon
bindkey "^R" history-incremental-search-backward
bindkey "^S" history-incremental-search-forward

# smart insert last word 直前のコマンドの最後の単語を一瞬で入力できる
autoload smart-insert-last-word
zle -N insert-lastword smart-insert-last-word
bindkey '^[.' insert-lastword

# 入力中のコマンドをviで編集できる。
autoload -Uz edit-command-line
zle -N edit-command-line
bindkey '^[e' edit-command-line

# 入力中のコマンドを一瞬どかすことができる
bindkey '^U' push-line

[ -f ~/.fzf.zsh ] && source ~/.fzf.zsh

# Directory Hash ディレクトリ名のエイリアスのようなもの
hash -d dot=/home/masa/Documents/dotfiles

setopt auto_cd

export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion
echo $HOME



HISTFILE=~/.zsh_history
HISTSIZE=10000
SAVEHIST=10000
setopt appendhistory          # 履歴を追記
setopt inc_append_history      # コマンド実行後に履歴を即保存
setopt sharehistory           # セッション間で履歴を共有
setopt histignoredups         # 重複する履歴を無視
