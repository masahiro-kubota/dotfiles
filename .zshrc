
# ~/.zshrc: executed by zsh for interactive shells.

# インタラクティブシェルでない場合は終了
if [[ $- != *i* ]]; then
  return
fi

HISTFILE=~/.zsh_history
HISTSIZE=10000
SAVEHIST=10000
setopt appendhistory          # 履歴を追記
setopt inc_append_history      # コマンド実行後に履歴を即保存
setopt sharehistory           # セッション間で履歴を共有
setopt histignoredups         # 重複する履歴を無視

# プロンプトの設定
PROMPT='%F{green}%n@%m%f:%F{blue}%~%f$ '


# カラー設定
if [ -x /usr/bin/dircolors ]; then
    [[ -r ~/.dircolors ]] && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
fi

# エイリアス
if [ -f ~/.aliases ]; then
    . ~/.aliases
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

# Git worktree selector
select_worktree() {
  local worktrees
  worktrees=$(git worktree list --porcelain | awk '/worktree / {print $2}')
  if [[ -z "$worktrees" ]]; then
    echo "No worktrees found."
    return 1
  fi
  local selected
  selected=$(echo "$worktrees" | fzf)
  if [[ -n "$selected" ]]; then
    echo "Switching to: $selected"
    cd "$selected"
  fi
}

# Ctrl+J でworktree選択
zle -N select_worktree
bindkey '^J' select_worktree

[ -f ~/.fzf.zsh ] && source ~/.fzf.zsh

# Directory Hash ディレクトリ名のエイリアスのようなもの
hash -d dot=/home/masa/Documents/dotfiles

setopt auto_cd

export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion




#eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"
#. "$HOME/.local/bin/env"

# ros2 setting
source /opt/ros/humble/setup.zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///opt/autoware/cyclonedds.xml
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
export GTEST_COLOR=1
eval "$(register-python-argcomplete3 colcon)"
eval "$(register-python-argcomplete ros2)"
alias srcws='source install/setup.zsh && eval "$(register-python-argcomplete3 ros2)" && eval "$(register-python-argcomplete3 colcon)"'

export DOTNET_ROOT="/usr/local/share/dotnet"
export PATH=$PATH:$DOTNET_ROOT

export PATH=$PATH:~/bin


function start_new_task() {
    local task_id=$1
    # mainを更新
    cd /workspace/main
    git pull origin main
    # 新しいworktreeを作成
    git worktree add ../task-${task_id} -b ai/task-${task_id}
    cd ../task-${task_id}
    git submodule update --init --recursive
    echo "Ready to work on task ${task_id}"
}


if [ -f /usr/share/fzf/key-bindings.zsh ]; then
  source /usr/share/fzf/key-bindings.zsh
elif [ -f /usr/share/doc/fzf/examples/key-bindings.zsh ]; then
  source /usr/share/doc/fzf/examples/key-bindings.zsh
fi
bindkey -e
bindkey '^R' fzf-history-widget

# セキュリティ診断＋HTMLレポート生成コマンド
nmapsec() {
  if [ -z "$1" ]; then
    echo "Usage: nmapsec <IP-or-hostname>"
    return 1
  fi

  local target="$1"
  local base_dir="$HOME/nmapsec_logs"

  mkdir -p "$base_dir/$target"
  local ts
  ts="$(date +'%Y%m%d-%H%M%S')"
  local prefix="$base_dir/$target/scan-sec-${target}-${ts}"

  echo "[*] Scanning $target ..."
  sudo nmap -sS -sV -sC --script vuln -O -T4 -oA "$prefix" "$target"

  echo "[*] Generating HTML report ..."
  xsltproc "${prefix}.xml" -o "${prefix}.html"

  echo "[*] Opening ${prefix}.html"
  xdg-open "${prefix}.html" >/dev/null 2>&1 &

  echo "[*] Done. Files are in: $base_dir/$target"
}

myssh() {
  if [ "$#" -ne 1 ]; then
    echo "Usage: myssh user@host"
    return 1
  fi

  local target="$1"
  local keyfile="$HOME/.ssh/id_ed25519"
  local pubfile="${keyfile}.pub"
  local local_user="$USER"
  local local_host="$(hostname -I | awk '{print $1}')"

  # --- 0. 公開鍵が無ければ作る ---
  if [ ! -f "$pubfile" ]; then
    echo "→ SSH鍵がありません。ed25519鍵を作成します..."
    ssh-keygen -t ed25519 -f "$keyfile" -C "$USER@$(hostname)" -N ""
    echo "✔ SSH鍵を作成しました：$pubfile"
  fi

  # --- 1. 公開鍵を読み込む ---
  local pubkey
  pubkey="$(< "$pubfile")"

  # --- 2. remote_aliases / aliases を送る ---
  scp -q ~/.kubota_aliases ~/.kubota_remote_aliases \
    "${target}:~/." || return 1

  # --- 3. 公開鍵を MY_PUBKEY として環境変数に渡しログイン ---
  # --- 3. 公開鍵を MY_PUBKEY として環境変数に渡し、専用 rc で bash 起動 ---
  ssh "$target" -t "
    export MY_PUBKEY='$pubkey'
    export PULL_LOCAL_USER='$local_user'
    export PULL_LOCAL_HOST='$local_host'

    # kubota 専用 rc をその場で生成
    cat > ~/.kubota_rc << 'EOF'
# まず元々の bashrc を読む
if [ -f ~/.bashrc ]; then
  . ~/.bashrc
fi

# kubota 用 aliases
if [ -f ~/.kubota_aliases ]; then
  . ~/.kubota_aliases
fi

# kubota 用 remote aliases（regkey など）
if [ -f ~/.kubota_remote_aliases ]; then
  . ~/.kubota_remote_aliases
fi
EOF

    # この rc を使って対話 bash を起動
    exec bash --rcfile ~/.kubota_rc -i
  "
}

